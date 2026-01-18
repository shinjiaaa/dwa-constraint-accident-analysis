"""
Run S1 or S3 with GUI + screenshots + operational logging.
"""

import json
import os
import sys
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import Physics

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.dwa import DWACandidateGenerator
from src.constraints import ConstraintAuditor
from src.explainer import DecisionExplainer
from src.operational_logging import OperationalLogger
from src.nlg import build_natural_language_summary
from src.llm import build_llm_prompt, call_openai_llm
from src.env import load_dotenv
from scenarios import get_scenario

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None


class ScenarioRunner:
    def __init__(
        self,
        scenario_name: str,
        seed: int,
        out_dir: str,
        frame_interval_s: float = 1.0,
        frame_width: int = 640,
        frame_height: int = 360,
        llm: bool = False,
        llm_model: str = "gpt-4o-mini",
        gui: bool = True,
        config_path: Optional[str] = None,
    ):
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        load_dotenv(os.path.join(project_root, ".env"))
        self.scenario = get_scenario(scenario_name)
        if config_path:
            self._apply_config_override(config_path)
        self.scenario.seed = seed
        self.out_dir = os.path.abspath(out_dir)
        self.gui = gui

        self.ctrl_freq = 240
        self.dt = 1.0 / self.ctrl_freq
        self.frame_interval_s = max(0.1, frame_interval_s)
        self.screenshot_interval_steps = max(1, int(self.frame_interval_s / self.dt))
        self.frame_width = max(160, frame_width)
        self.frame_height = max(90, frame_height)

        np.random.seed(seed)

        self.dwa = DWACandidateGenerator(
            v_min=self.scenario.v_min,
            v_max=self.scenario.v_max,
            yaw_rate_min=self.scenario.yaw_rate_min,
            yaw_rate_max=self.scenario.yaw_rate_max,
            v_resolution=5,
            yaw_rate_resolution=5,
            seed=seed,
        )
        self.auditor = ConstraintAuditor(config=self.scenario.constraint_config)
        self.explainer = DecisionExplainer(self.dwa, self.auditor)

        self.env = None
        self.obstacle_ids = []

        # Output structure
        self.frames_dir = os.path.join(self.out_dir, self.scenario.name, "frames")
        self.logs_dir = os.path.join(self.out_dir, self.scenario.name)
        os.makedirs(self.frames_dir, exist_ok=True)
        os.makedirs(self.logs_dir, exist_ok=True)
        self.logger = OperationalLogger(self.logs_dir)
        self.report_path = os.path.join(self.logs_dir, "report.json")
        api_key = os.environ.get("OPENAI_API_KEY")
        self.llm_enabled = bool(llm or api_key)
        self.llm_model = llm_model
        if api_key:
            masked = f"{api_key[:6]}...{api_key[-4:]}" if len(api_key) > 12 else "set"
        else:
            masked = "missing"
        print(f"[LLM] enabled={self.llm_enabled}, key={masked}, model={self.llm_model}, flag={llm}")

        self.ring_buffer = deque(maxlen=int(5.0 / self.dt))
        self.no_feasible_steps = 0
        self.last_event = None
        self.llm_triggered = False
    def _apply_config_override(self, config_path: str):
        """Override S1 config from a JSON file."""
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)

        # Enforce no reverse for S1
        self.scenario.v_min = float(cfg.get("v_min", 0.0))

        # Override initial speed (x)
        init_speed = cfg.get("initial_speed")
        if init_speed is not None:
            self.scenario.initial_velocity[0] = float(init_speed)

        # Override obstacle x (first obstacle)
        obstacle_x = cfg.get("obstacle_x")
        if obstacle_x is not None and self.scenario.obstacles:
            self.scenario.obstacles[0][0] = float(obstacle_x)

        # Override constraint limits
        max_accel = cfg.get("max_acceleration")
        if max_accel is not None:
            self.scenario.constraint_config.max_acceleration = float(max_accel)
        max_yaw = cfg.get("max_yaw_rate")
        if max_yaw is not None:
            self.scenario.constraint_config.max_yaw_rate = float(max_yaw)

        # Optional overrides
        if "max_steps" in cfg:
            self.scenario.max_steps = int(cfg["max_steps"])
        if "candidate_count" in cfg:
            self.scenario.candidate_count = int(cfg["candidate_count"])


    def _create_env(self):
        self.env = CtrlAviary(
            num_drones=1,
            gui=self.gui,
            physics=Physics.PYB,
            pyb_freq=self.ctrl_freq,
            ctrl_freq=self.ctrl_freq,
            initial_xyzs=np.array([self.scenario.initial_position]),
            initial_rpys=np.array([[0.0, 0.0, 0.0]]),
        )

        obs, _ = self.env.reset(seed=self.scenario.seed)
        state = obs[0]

        # Set initial velocity
        if hasattr(self.env, "DRONE_IDS"):
            p.resetBaseVelocity(
                self.env.DRONE_IDS[0],
                linearVelocity=self.scenario.initial_velocity.tolist(),
                angularVelocity=[0, 0, 0],
                physicsClientId=self.env.CLIENT
            )
            state[3:6] = self.scenario.initial_velocity

        # Add obstacle bodies (visual + collision)
        self._spawn_obstacles()

        return state

    def _spawn_obstacles(self):
        self.obstacle_ids = []
        for pos in self.scenario.obstacles:
            shape = getattr(self.scenario, "obstacle_shape", "sphere")
            size = getattr(self.scenario, "obstacle_size", {"radius": 0.15})
            if shape in ("box", "wall"):
                half_extents = size.get("half_extents", [0.1, 0.1, 0.1])
                visual_id = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=half_extents,
                    rgbaColor=[1, 0, 0, 0.9],
                    physicsClientId=self.env.CLIENT
                )
                collision_id = p.createCollisionShape(
                    p.GEOM_BOX,
                    halfExtents=half_extents,
                    physicsClientId=self.env.CLIENT
                )
            else:
                radius = size.get("radius", 0.15)
                visual_id = p.createVisualShape(
                    p.GEOM_SPHERE,
                    radius=radius,
                    rgbaColor=[1, 0, 0, 0.9],
                    physicsClientId=self.env.CLIENT
                )
                collision_id = p.createCollisionShape(
                    p.GEOM_SPHERE,
                    radius=radius,
                    physicsClientId=self.env.CLIENT
                )
            body_id = p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision_id,
                baseVisualShapeIndex=visual_id,
                basePosition=pos.tolist(),
                physicsClientId=self.env.CLIENT
            )
            self.obstacle_ids.append(body_id)

    def _compute_dist_min_ttc(self, position: np.ndarray, velocity: np.ndarray) -> Tuple[float, float]:
        if not self.scenario.obstacles:
            return float("inf"), float("inf")
        min_dist = float("inf")
        min_ttc = float("inf")
        for obs_pos in self.scenario.obstacles:
            rel = obs_pos - position
            dist = np.linalg.norm(rel)
            min_dist = min(min_dist, dist)
            if dist < 1e-6:
                min_ttc = 0.0
                continue
            # Closing speed should be positive when moving toward obstacle
            closing_speed = np.dot(rel, velocity) / dist
            if closing_speed > 1e-6:
                ttc = max(
                    0.0,
                    (dist - self.scenario.constraint_config.min_obstacle_distance) / closing_speed
                )
                min_ttc = min(min_ttc, ttc)
        return min_dist, min_ttc

    def _check_collision(self) -> bool:
        if not hasattr(self.env, "DRONE_IDS"):
            return False
        drone_id = self.env.DRONE_IDS[0]
        for obs_id in self.obstacle_ids:
            if p.getContactPoints(bodyA=drone_id, bodyB=obs_id, physicsClientId=self.env.CLIENT):
                return True
        return False

    def _camera_matrices(self, target: np.ndarray, mode: str = "close"):
        width, height = self.frame_width, self.frame_height
        if mode == "drone":
            # First-person: camera at drone, looking forward
            if hasattr(self.env, "DRONE_IDS"):
                drone_id = self.env.DRONE_IDS[0]
                pos, quat = p.getBasePositionAndOrientation(
                    drone_id, physicsClientId=self.env.CLIENT
                )
                rot = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
                forward = rot @ np.array([1.0, 0.0, 0.0])
                up = rot @ np.array([0.0, 0.0, 1.0])
                eye = np.array(pos) + 0.01 * up
                target_pos = eye + 0.6 * forward
                view = p.computeViewMatrix(
                    cameraEyePosition=eye.tolist(),
                    cameraTargetPosition=target_pos.tolist(),
                    cameraUpVector=up.tolist()
                )
            else:
                view = p.computeViewMatrixFromYawPitchRoll(
                    cameraTargetPosition=target.tolist(),
                    distance=0.5,
                    yaw=0,
                    pitch=-10,
                    roll=0,
                    upAxisIndex=2
                )
        else:
            # Third-person: frame both drone and obstacle in view
            view = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=target.tolist(),
                distance=1.4,
                yaw=30,
                pitch=-18,
                roll=0,
                upAxisIndex=2
            )
        proj = p.computeProjectionMatrixFOV(
            fov=45,
            aspect=width / height,
            nearVal=0.1,
            farVal=50.0
        )
        return width, height, view, proj

    def _save_frame(self, step: int, tag: str, target: np.ndarray, mode: str) -> str:
        width, height, view, proj = self._camera_matrices(target, mode=mode)
        _, _, rgba, _, _ = p.getCameraImage(
            width=width,
            height=height,
            viewMatrix=view,
            projectionMatrix=proj,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        img = np.reshape(rgba, (height, width, 4))[:, :, :3]
        filename = f"{self.scenario.name}_step{step:05d}_{tag}_{mode}.png"
        path = os.path.join(self.frames_dir, filename)
        if plt is not None:
            plt.imsave(path, img)
        else:
            # Fallback PPM
            ppm_path = path.replace(".png", ".ppm")
            with open(ppm_path, "wb") as f:
                f.write(f"P6 {width} {height} 255\n".encode("ascii"))
                f.write(img.astype(np.uint8).tobytes())
            path = ppm_path
        return path

    def _build_cost_terms(self, candidate, current_state, weights):
        vx, vy, vz, yaw_rate = candidate["action"]
        dt = 0.1
        current_pos = current_state[:3]
        predicted_vel = np.array([vx, vy, vz])
        predicted_pos = current_pos + predicted_vel * dt

        goal_dist = np.linalg.norm(predicted_pos - self.scenario.goal)
        obstacle_penalty = 0.0
        if self.scenario.obstacles:
            min_obstacle_dist = min(
                np.linalg.norm(predicted_pos - obs) for obs in self.scenario.obstacles
            )
            if min_obstacle_dist < 1.0:
                obstacle_penalty = np.exp(-min_obstacle_dist)

        velocity_mag = np.linalg.norm(predicted_vel)
        yaw_mag = abs(yaw_rate)

        terms = {
            "goal_distance": goal_dist,
            "obstacle_penalty": obstacle_penalty,
            "velocity_magnitude": velocity_mag,
            "yaw_rate_magnitude": yaw_mag,
        }

        total = (
            weights["goal_distance"] * terms["goal_distance"] +
            weights["obstacle"] * terms["obstacle_penalty"] +
            weights["velocity"] * (terms["velocity_magnitude"] ** 2) +
            weights["yaw_rate"] * (terms["yaw_rate_magnitude"] ** 2)
        )

        return terms, total

    def _build_top_k(self, candidates, cost_expl, feasibility, k=5):
        id_to_cand = {c["id"]: c for c in candidates}
        id_to_audit = {r.candidate_id: r for r in feasibility.candidate_results}
        top = []
        for cand_id, cost in cost_expl["scores"][:k]:
            audit = id_to_audit[cand_id]
            top.append({
                "candidate_id": cand_id,
                "action": id_to_cand[cand_id]["action"].tolist(),
                "cost_total": cost,
                "is_feasible": audit.is_feasible,
                "slack_vector": {v.constraint_name: -float(v.violation_amount) for v in audit.violations},
            })
        return top

    def _velocity_to_rpm(self, velocity_cmd: np.ndarray, yaw_rate: float) -> np.ndarray:
        base_rpm = float(self.env.HOVER_RPM) if hasattr(self.env, "HOVER_RPM") else 400.0
        vx, vy, vz = velocity_cmd
        rpm_gain_xy = 50.0
        rpm_gain_z = 100.0
        rpm1 = base_rpm + vx * rpm_gain_xy + vy * rpm_gain_xy + vz * rpm_gain_z
        rpm2 = base_rpm + vx * rpm_gain_xy - vy * rpm_gain_xy - vz * rpm_gain_z
        rpm3 = base_rpm - vx * rpm_gain_xy - vy * rpm_gain_xy + vz * rpm_gain_z
        rpm4 = base_rpm - vx * rpm_gain_xy + vy * rpm_gain_xy - vz * rpm_gain_z
        yaw_scale = yaw_rate * rpm_gain_xy
        rpm1 += yaw_scale
        rpm2 -= yaw_scale
        rpm3 += yaw_scale
        rpm4 -= yaw_scale
        rpm = np.array([rpm1, rpm2, rpm3, rpm4])
        if hasattr(self.env, "MAX_RPM"):
            rpm = np.clip(rpm, 0, float(self.env.MAX_RPM))
        return rpm

    def _third_person_target(self, drone_pos: np.ndarray) -> np.ndarray:
        if self.scenario.obstacles:
            nearest = min(self.scenario.obstacles, key=lambda o: np.linalg.norm(o - drone_pos))
            return (drone_pos + nearest) / 2.0
        return drone_pos

    def run(self):
        state = self._create_env()
        start_time = time.time()

        for step in range(self.scenario.max_steps):
            # Generate candidates & audit
            candidates = self.dwa.generate_candidates(state, num_candidates=self.scenario.candidate_count)
            feasibility = self.auditor.audit_feasible_set(
                candidates, state, self.scenario.obstacles, dt=self.dt
            )
            best, cost_expl = self.dwa.select_best_candidate(
                candidates, state, self.scenario.goal, self.scenario.obstacles, self.scenario.cost_weights
            )

            # Metrics
            position = state[:3]
            velocity = state[3:6]
            dist_min, ttc = self._compute_dist_min_ttc(position, velocity)
            selected_audit = next(
                r for r in feasibility.candidate_results if r.candidate_id == best["id"]
            )
            selected_slack = {v.constraint_name: -float(v.violation_amount) for v in selected_audit.violations}
            cost_terms, cost_total = self._build_cost_terms(best, state, self.scenario.cost_weights)

            # Logging (tick)
            tick_entry = {
                "time_s": time.time() - start_time,
                "timestep": step,
                "scenario": self.scenario.name,
                "gt_label": self.scenario.gt_label,
                "state": {
                    "position": position.tolist(),
                    "velocity": velocity.tolist(),
                    "attitude_rpy": state[7:10].tolist() if len(state) >= 10 else None,
                },
                "selected_action": best["action"].tolist(),
                "metrics": {
                    "dist_min": dist_min,
                    "ttc": ttc,
                    "candidate_count": len(candidates),
                    "n_feasible": feasibility.n_feasible,
                    "feasible_ratio": feasibility.feasible_ratio,
                    "decision_margin": cost_expl.get("top_2_margin"),
                },
                "dwa": {
                    "selected_cost_total": cost_total,
                    "cost_terms": cost_terms,
                },
                "constraints": {
                    "per_constraint_stats": feasibility.per_constraint_stats,
                    "selected_slack": selected_slack,
                },
            }
            if feasibility.minimal_relaxation is not None:
                tick_entry["constraints"]["minimal_relaxation"] = feasibility.minimal_relaxation

            self.logger.log_tick(tick_entry)
            self.ring_buffer.append(tick_entry)

            # Screenshots every interval
            if step % self.screenshot_interval_steps == 0:
                third_target = self._third_person_target(position)
                self._save_frame(step, "periodic", third_target, mode="close")

            # Trigger logic
            collision = self._check_collision()
            if feasibility.n_feasible == 0:
                self.no_feasible_steps += 1
            else:
                self.no_feasible_steps = 0

            trigger_s1 = (
                self.scenario.name == "s1"
                and (
                    (self.no_feasible_steps * self.dt) >= 0.3
                    or (ttc < 1.0)
                    or collision
                )
            )
            trigger_s3 = (
                self.scenario.name == "s3"
                and (dist_min < 0.5)
                and (feasibility.n_feasible > 0)
            )

            if trigger_s1 or trigger_s3:
                third_target = self._third_person_target(position)
                trigger_paths = [
                    self._save_frame(step, "trigger", third_target, mode="close"),
                    self._save_frame(step, "trigger", position, mode="drone"),
                ]
                event_entry = {
                    "time_s": time.time() - start_time,
                    "timestep": step,
                    "scenario": self.scenario.name,
                    "gt_label": self.scenario.gt_label,
                    "reason": "trigger",
                    "ring_buffer": list(self.ring_buffer),
                    "top_k": self._build_top_k(candidates, cost_expl, feasibility, k=5),
                    "constraints": {
                        "per_constraint_stats": feasibility.per_constraint_stats,
                        "minimal_relaxation": feasibility.minimal_relaxation,
                    },
                    "metrics": {
                        "dist_min": dist_min,
                        "ttc": ttc,
                        "n_feasible": feasibility.n_feasible,
                    },
                    "screenshots": trigger_paths,
                }
                # Build natural language summary at trigger
                report_stub = {
                    "scenario": self.scenario.name,
                    "gt_label": self.scenario.gt_label,
                    "predicted_label": self.scenario.gt_label,
                    "expected_xai": getattr(self.scenario, "expected_xai", None),
                }
                event_entry["summary_text"] = build_natural_language_summary(
                    report_stub,
                    last_event=event_entry,
                    last_tick=self.ring_buffer[-1] if self.ring_buffer else None,
                    language="ko",
                )
                if not self.llm_enabled:
                    print("\n[NLG Trigger Summary]")
                    print(event_entry["summary_text"])
                # Optional LLM summary at trigger (only once)
                if self.llm_enabled and not self.llm_triggered:
                    llm_prompt = build_llm_prompt(
                        report_stub,
                        event_entry,
                        self.ring_buffer[-1] if self.ring_buffer else None,
                    )
                    llm_text = call_openai_llm(llm_prompt, model=self.llm_model)
                    event_entry["llm_summary_text"] = llm_text
                    print("\n[LLM Trigger Summary]")
                    print(llm_text)
                    self.llm_triggered = True
                elif not self.llm_enabled:
                    print("\n[LLM Trigger Summary]")
                    print("[LLM] disabled (KEY missing or --llm not set).")
                self.logger.log_event(event_entry)
                self.last_event = event_entry

            # Step environment
            vx, vy, vz, yaw_rate = best["action"]
            rpm = self._velocity_to_rpm(np.array([vx, vy, vz]), yaw_rate)
            obs, _, terminated, truncated, _ = self.env.step(rpm.reshape(1, -1))
            state = obs[0]

            if terminated or truncated:
                break

        # Keep GUI open for screenshots
        try:
            input("Simulation finished. Press Enter to close the GUI.")
        except KeyboardInterrupt:
            pass

        self.env.close()

        # Write compact report
        report = {
            "scenario": self.scenario.name,
            "gt_label": self.scenario.gt_label,
            "predicted_label": self.scenario.gt_label,
            "expected_xai": getattr(self.scenario, "expected_xai", None),
            "summary_text": build_natural_language_summary(
                {
                    "scenario": self.scenario.name,
                    "gt_label": self.scenario.gt_label,
                    "predicted_label": self.scenario.gt_label,
                    "expected_xai": getattr(self.scenario, "expected_xai", None),
                },
                last_event=self.last_event,
                last_tick=self.ring_buffer[-1] if self.ring_buffer else None,
                language="ko",
            ),
        }
        with open(self.report_path, "w", encoding="utf-8") as f:
            import json
            json.dump(report, f, ensure_ascii=False, indent=2)
        if not self.llm_enabled:
            print("\n[NLG Final Summary]")
            print(report["summary_text"])

        # Optional LLM summary
        if self.llm_enabled:
            prompt = build_llm_prompt(report, self.last_event, self.ring_buffer[-1] if self.ring_buffer else None)
            llm_text = call_openai_llm(prompt, model=self.llm_model)
            report["llm_summary_text"] = llm_text
            with open(self.report_path, "w", encoding="utf-8") as f:
                import json
                json.dump(report, f, ensure_ascii=False, indent=2)
            print("\n[LLM Summary]")
            print(llm_text)


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Run scenario (s1 or s3) with GUI.")
    parser.add_argument("--scenario", required=True, choices=["s1", "s3"])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", type=str, default="outputs")
    parser.add_argument("--frame-interval", type=float, default=1.0)
    parser.add_argument("--frame-width", type=int, default=640)
    parser.add_argument("--frame-height", type=int, default=360)
    parser.add_argument("--llm", action="store_true", default=False, help="Enable LLM summary (OpenAI)")
    parser.add_argument("--llm-model", type=str, default="gpt-4o-mini")
    parser.add_argument("--config", type=str, default=None, help="Override scenario config JSON")
    parser.add_argument("--gui", action="store_true", default=False, help="Force GUI on")
    parser.add_argument("--headless", action="store_true", default=False, help="Force headless")
    args = parser.parse_args()

    gui_enabled = True
    if args.headless:
        gui_enabled = False
    elif args.gui:
        gui_enabled = True

    runner = ScenarioRunner(
        args.scenario,
        args.seed,
        args.out,
        frame_interval_s=args.frame_interval,
        frame_width=args.frame_width,
        frame_height=args.frame_height,
        llm=args.llm,
        llm_model=args.llm_model,
        gui=gui_enabled,
        config_path=args.config,
    )
    runner.run()


if __name__ == "__main__":
    exit(main())
