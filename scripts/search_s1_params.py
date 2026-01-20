"""
Grid search for strong S1 (INEVITABLE_DYNAMICS) configurations.
"""

import json
import os
import sys
from contextlib import redirect_stdout, redirect_stderr
from itertools import product
from typing import Dict, Optional

import numpy as np
import pybullet as p
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import Physics

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.dwa import DWACandidateGenerator
from src.constraints import ConstraintAuditor
from scenarios.s1_inevitable_dynamics import build_s1


def spawn_obstacles(env: CtrlAviary, obstacles, shape: str, size: dict):
    ids = []
    for pos in obstacles:
        if shape in ("box", "wall"):
            half_extents = size.get("half_extents", [0.1, 0.1, 0.1])
            visual_id = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=half_extents,
                rgbaColor=[1, 0, 0, 0.9],
                physicsClientId=env.CLIENT
            )
            collision_id = p.createCollisionShape(
                p.GEOM_BOX,
                halfExtents=half_extents,
                physicsClientId=env.CLIENT
            )
        else:
            radius = size.get("radius", 0.15)
            visual_id = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=radius,
                rgbaColor=[1, 0, 0, 0.9],
                physicsClientId=env.CLIENT
            )
            collision_id = p.createCollisionShape(
                p.GEOM_SPHERE,
                radius=radius,
                physicsClientId=env.CLIENT
            )
        body_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_id,
            baseVisualShapeIndex=visual_id,
            basePosition=pos.tolist(),
            physicsClientId=env.CLIENT
        )
        ids.append(body_id)
    return ids


def check_collision(env: CtrlAviary, obstacle_ids) -> bool:
    if not hasattr(env, "DRONE_IDS"):
        return False
    drone_id = env.DRONE_IDS[0]
    for obs_id in obstacle_ids:
        if p.getContactPoints(bodyA=drone_id, bodyB=obs_id, physicsClientId=env.CLIENT):
            return True
    return False


def compute_dist_min_ttc(position: np.ndarray, velocity: np.ndarray, obstacles, d_safe: float):
    if not obstacles:
        return float("inf"), float("inf")
    min_dist = float("inf")
    min_ttc = float("inf")
    for obs_pos in obstacles:
        rel = obs_pos - position
        dist = np.linalg.norm(rel)
        min_dist = min(min_dist, dist)
        if dist < 1e-6:
            min_ttc = 0.0
            continue
        closing_speed = np.dot(rel, velocity) / dist
        if closing_speed > 1e-6:
            ttc = max(0.0, (dist - d_safe) / closing_speed)
            min_ttc = min(min_ttc, ttc)
    return min_dist, min_ttc


def run_combo(initial_speed, obstacle_x, max_accel, max_yaw_rate, quiet: bool, max_steps: int) -> Dict:
    cfg = build_s1()
    cfg.v_min = 0.0
    cfg.initial_velocity[0] = float(initial_speed)
    cfg.obstacles[0][0] = float(obstacle_x)
    cfg.constraint_config.max_acceleration = float(max_accel)
    cfg.constraint_config.max_yaw_rate = float(max_yaw_rate)
    cfg.max_steps = max_steps

    # Initialize modules
    dwa = DWACandidateGenerator(
        v_min=cfg.v_min,
        v_max=cfg.v_max,
        yaw_rate_min=cfg.yaw_rate_min,
        yaw_rate_max=cfg.yaw_rate_max,
        v_resolution=5,
        yaw_rate_resolution=5,
        seed=cfg.seed,
    )
    auditor = ConstraintAuditor(config=cfg.constraint_config)

    # Environment (headless)
    if quiet:
        with open(os.devnull, "w") as devnull, redirect_stdout(devnull), redirect_stderr(devnull):
            env = CtrlAviary(
                num_drones=1,
                gui=False,
                physics=Physics.PYB,
                pyb_freq=240,
                ctrl_freq=240,
                initial_xyzs=np.array([cfg.initial_position]),
                initial_rpys=np.array([[0.0, 0.0, 0.0]]),
            )
            obs, _ = env.reset(seed=cfg.seed)
    else:
        env = CtrlAviary(
            num_drones=1,
            gui=False,
            physics=Physics.PYB,
            pyb_freq=240,
            ctrl_freq=240,
            initial_xyzs=np.array([cfg.initial_position]),
            initial_rpys=np.array([[0.0, 0.0, 0.0]]),
        )
        obs, _ = env.reset(seed=cfg.seed)
    state = obs[0]
    if hasattr(env, "DRONE_IDS"):
        p.resetBaseVelocity(
            env.DRONE_IDS[0],
            linearVelocity=cfg.initial_velocity.tolist(),
            angularVelocity=[0, 0, 0],
            physicsClientId=env.CLIENT
        )
        state[3:6] = cfg.initial_velocity

    shape = getattr(cfg, "obstacle_shape", "sphere")
    size = getattr(cfg, "obstacle_size", {"radius": 0.15})
    obstacle_ids = spawn_obstacles(env, cfg.obstacles, shape, size)

    collapse_start = None
    longest_streak = 0
    current_streak = 0
    dominant_constraint = None
    delta_accel = None
    collided = False
    near_miss = False

    for step in range(cfg.max_steps):
        candidates = dwa.generate_candidates(state, num_candidates=cfg.candidate_count)
        feasibility = auditor.audit_feasible_set(candidates, state, cfg.obstacles, dt=1/240)

        position = state[:3]
        velocity = state[3:6]
        dist_min, _ = compute_dist_min_ttc(position, velocity, cfg.obstacles, cfg.constraint_config.min_obstacle_distance)

        if dist_min < 0.5:
            near_miss = True

        if feasibility.n_feasible == 0:
            if collapse_start is None:
                collapse_start = step
            current_streak += 1
            longest_streak = max(longest_streak, current_streak)
            # dominant constraint by violation_ratio
            per_stats = feasibility.per_constraint_stats
            if per_stats:
                dominant_constraint = max(per_stats.items(), key=lambda x: x[1].get("violation_ratio", 0.0))[0]
            if feasibility.minimal_relaxation:
                delta_accel = feasibility.minimal_relaxation.get("max_acceleration", delta_accel)
        else:
            current_streak = 0

        # step env with selected action
        best, _ = dwa.select_best_candidate(candidates, state, cfg.goal, cfg.obstacles, cfg.cost_weights)
        vx, vy, vz, yaw_rate = best["action"]
        base_rpm = float(env.HOVER_RPM) if hasattr(env, "HOVER_RPM") else 400.0
        rpm_gain_xy = 50.0
        rpm_gain_z = 100.0
        rpm1 = base_rpm + vx * rpm_gain_xy + vy * rpm_gain_xy + vz * rpm_gain_z
        rpm2 = base_rpm + vx * rpm_gain_xy - vy * rpm_gain_xy - vz * rpm_gain_z
        rpm3 = base_rpm - vx * rpm_gain_xy - vy * rpm_gain_xy + vz * rpm_gain_z
        rpm4 = base_rpm - vx * rpm_gain_xy + vy * rpm_gain_xy - vz * rpm_gain_z
        rpm = np.array([rpm1, rpm2, rpm3, rpm4])
        if hasattr(env, "MAX_RPM"):
            rpm = np.clip(rpm, 0, float(env.MAX_RPM))

        obs, _, terminated, truncated, _ = env.step(rpm.reshape(1, -1))
        state = obs[0]

        if check_collision(env, obstacle_ids):
            collided = True
            break
        if terminated or truncated:
            break

    env.close()

    return {
        "initial_speed": initial_speed,
        "obstacle_x": obstacle_x,
        "max_acceleration": max_accel,
        "max_yaw_rate": max_yaw_rate,
        "collided": collided,
        "near_miss": near_miss,
        "first_collapse_step": collapse_start,
        "collapse_duration_steps": longest_streak,
        "dominant_constraint": dominant_constraint,
        "delta_accel": delta_accel,
    }


def score_config(result: Dict, dt: float) -> float:
    if not (result["collided"] or result["near_miss"]):
        return -1.0
    if result["collapse_duration_steps"] is None:
        return -1.0
    if result["collapse_duration_steps"] * dt < 0.3:
        return -1.0
    delta_accel = result["delta_accel"] or 0.0
    return result["collapse_duration_steps"] + 10.0 * delta_accel


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--quiet", action="store_true", default=False, help="Reduce simulator logs")
    parser.add_argument("--max-steps", type=int, default=240, help="Max steps per combo")
    parser.add_argument("--max-combos", type=int, default=None, help="Limit number of combos (for quick test)")
    parser.add_argument("--progress-every", type=int, default=20, help="Print progress every N combos")
    args = parser.parse_args()
    initial_speeds = [2.0, 2.5, 3.0, 3.5]
    obstacle_xs = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0]
    max_accels = [0.3, 0.4, 0.6, 0.8, 1.0]
    max_yaws = [0.2, 0.3, 0.4, 0.6, 0.8]

    results = []
    dt = 1 / 240
    combos = list(product(initial_speeds, obstacle_xs, max_accels, max_yaws))
    if args.max_combos is not None:
        combos = combos[: args.max_combos]
    total = len(combos)
    for idx, combo in enumerate(combos, start=1):
        res = run_combo(*combo, quiet=args.quiet, max_steps=args.max_steps)
        res["score"] = score_config(res, dt)
        results.append(res)
        if idx % args.progress_every == 0 or idx == total:
            print(f"[progress] {idx}/{total}")

    results.sort(key=lambda r: r["score"], reverse=True)

    print("\nTop 10 configs:")
    header = "speed  obs_x  max_acc  max_yaw  collided  near_miss  first0  streak  dom  d_accel  score"
    print(header)
    for r in results[:10]:
        print(
            f"{r['initial_speed']:>4.1f}  {r['obstacle_x']:>4.1f}  {r['max_acceleration']:>6.2f}  "
            f"{r['max_yaw_rate']:>6.2f}  {str(r['collided']):>7}  {str(r['near_miss']):>9}  "
            f"{str(r['first_collapse_step']):>6}  {r['collapse_duration_steps']:>6}  "
            f"{str(r['dominant_constraint']):>4}  {str(r['delta_accel']):>7}  {r['score']:.2f}"
        )

    best = results[0]
    out_path = os.path.join("outputs", "s1_best_config.json")
    os.makedirs("outputs", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(best, f, ensure_ascii=False, indent=2)
    print(f"\nBest config saved to {out_path}")


if __name__ == "__main__":
    main()
