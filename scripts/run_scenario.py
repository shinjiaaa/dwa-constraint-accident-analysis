"""
Main execution script for running accident analysis scenarios
Integrates DWA, constraint-based XAI, and logging
"""

import numpy as np
import sys
import os
import time
from typing import List
import pybullet as p

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from src.dwa import DWACandidateGenerator
from src.constraints import ConstraintAuditor
from src.explainer import DecisionExplainer
from src.logger import JSONLLogger
from src.scenarios import ScenarioFactory
from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import Physics


class AccidentAnalysisRunner:
    """Main runner for accident analysis scenarios"""
    
    def __init__(
        self,
        scenario_name: str = 'A',
        headless: bool = True,
        real_time: bool = False,
        hold: bool = False,
        rpm_gain_xy: float = 50.0,
        rpm_gain_z: float = 100.0
    ):
        """
        Initialize runner
        
        Args:
            scenario_name: Scenario identifier ('A', 'B', or 'C')
            headless: Run without GUI
        """
        # Load scenario
        self.scenario = ScenarioFactory.get_scenario(scenario_name)
        self.headless = headless
        self.real_time = real_time
        self.hold = hold
        self.rpm_gain_xy = rpm_gain_xy
        self.rpm_gain_z = rpm_gain_z
        
        # Initialize components
        self.dwa_generator = DWACandidateGenerator(
            v_min=-2.0,
            v_max=2.0,
            yaw_rate_min=-1.0,
            yaw_rate_max=1.0,
            v_resolution=5,
            yaw_rate_resolution=5,
            seed=42
        )
        
        self.constraint_auditor = ConstraintAuditor(
            config=self.scenario.constraint_config
        )
        
        self.explainer = DecisionExplainer(
            dwa_generator=self.dwa_generator,
            constraint_auditor=self.constraint_auditor
        )
        
        # Logger (creates timestamped files)
        log_dir = os.path.join(os.path.dirname(__file__), '..', 'logs')
        self.logger = JSONLLogger(log_dir=log_dir)
        
        # Environment (will be created in run)
        self.env = None
        self.current_state = None
    
    def _velocity_to_rpm(self, velocity_cmd: np.ndarray, yaw_rate: float) -> np.ndarray:
        """
        Convert velocity command to RPM commands
        Simplified conversion (in practice, use proper control system)
        
        Args:
            velocity_cmd: [vx, vy, vz] velocity command
            yaw_rate: Yaw rate command
        
        Returns:
            RPM commands [rpm1, rpm2, rpm3, rpm4]
        """
        # Use hover RPM from the environment if available
        if self.env is not None and hasattr(self.env, "HOVER_RPM"):
            base_rpm = float(self.env.HOVER_RPM)
        else:
            base_rpm = 400.0
        vx, vy, vz = velocity_cmd
        
        # Scale RPM based on velocity (rough approximation)
        rpm1 = base_rpm + vx * self.rpm_gain_xy + vy * self.rpm_gain_xy + vz * self.rpm_gain_z
        rpm2 = base_rpm + vx * self.rpm_gain_xy - vy * self.rpm_gain_xy - vz * self.rpm_gain_z
        rpm3 = base_rpm - vx * self.rpm_gain_xy - vy * self.rpm_gain_xy + vz * self.rpm_gain_z
        rpm4 = base_rpm - vx * self.rpm_gain_xy + vy * self.rpm_gain_xy - vz * self.rpm_gain_z
        
        # Add yaw rate component
        yaw_scale = yaw_rate * self.rpm_gain_xy
        rpm1 += yaw_scale
        rpm2 -= yaw_scale
        rpm3 += yaw_scale
        rpm4 -= yaw_scale
        
        # Clip to reasonable range
        rpm = np.array([rpm1, rpm2, rpm3, rpm4])
        if self.env is not None and hasattr(self.env, "MAX_RPM"):
            rpm = np.clip(rpm, 0, float(self.env.MAX_RPM))
        else:
            rpm = np.clip(rpm, 0, 25000)
        
        return rpm

    def _add_visual_obstacles(self, obstacles: List[np.ndarray]):
        """Add simple visual obstacles for GUI inspection."""
        if self.env is None or not obstacles:
            return
        for obs_pos in obstacles:
            # Simple sphere obstacle
            radius = 0.15
            visual_id = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=radius,
                rgbaColor=[1, 0, 0, 0.8],
                physicsClientId=self.env.CLIENT
            )
            collision_id = p.createCollisionShape(
                p.GEOM_SPHERE,
                radius=radius,
                physicsClientId=self.env.CLIENT
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision_id,
                baseVisualShapeIndex=visual_id,
                basePosition=obs_pos.tolist(),
                physicsClientId=self.env.CLIENT
            )
    
    def _get_state_from_obs(self, obs: np.ndarray) -> np.ndarray:
        """
        Extract state vector from observation
        
        Observation format: [x, y, z, vx, vy, vz, ...]
        """
        return obs[0]  # First drone
    
    def _check_near_collision(
        self,
        position: np.ndarray,
        obstacles: List[np.ndarray],
        threshold: float = 0.3
    ) -> bool:
        """Check if drone is near collision"""
        for obs_pos in obstacles:
            dist = np.linalg.norm(position - obs_pos)
            if dist < threshold:
                return True
        return False
    
    def run(self):
        """Run the scenario"""
        print("=" * 80)
        print(f"Running Scenario: {self.scenario.name}")
        print(f"Description: {self.scenario.description}")
        print(f"Ground Truth Cause: {self.scenario.ground_truth_cause}")
        print("=" * 80)
        
        # Create environment
        self.env = CtrlAviary(
            num_drones=1,
            gui=not self.headless,
            physics=Physics.PYB,
            pyb_freq=240,
            ctrl_freq=240,
            initial_xyzs=np.array([self.scenario.initial_position]),
            initial_rpys=np.array([[0.0, 0.0, 0.0]]),
        )
        
        # Reset environment
        obs, info = self.env.reset(seed=42)
        self.current_state = self._get_state_from_obs(obs)

        # Apply initial velocity if specified
        if hasattr(self.env, "DRONE_IDS"):
            p.resetBaseVelocity(
                self.env.DRONE_IDS[0],
                linearVelocity=self.scenario.initial_velocity.tolist(),
                angularVelocity=[0, 0, 0],
                physicsClientId=self.env.CLIENT
            )
            # Update current state velocity to match initial velocity
            self.current_state[3:6] = self.scenario.initial_velocity
        
        # For scenario C: obstacles appear late
        dynamic_obstacles = self.scenario.obstacles.copy()
        if self.scenario.name == "C_delayed_response":
            # Start with no obstacles, add at step 30
            dynamic_obstacles = []

        # Add visual obstacles for GUI
        if not self.headless:
            self._add_visual_obstacles(dynamic_obstacles)
        
        print(f"\nInitial State:")
        print(f"  Position: {self.current_state[:3]}")
        print(f"  Velocity: {self.current_state[3:6]}")
        print(f"  Goal: {self.scenario.goal}")
        print(f"  Obstacles: {len(dynamic_obstacles)}")
        print()
        
        # Main simulation loop
        for step in range(self.scenario.max_steps):
            # For scenario C: add obstacle at step 30
            if self.scenario.name == "C_delayed_response" and step == 30:
                dynamic_obstacles = self.scenario.obstacles.copy()
                print(f"\n[Step {step}] Obstacle appeared (delayed detection)")
                if not self.headless:
                    self._add_visual_obstacles(dynamic_obstacles)
            
            # Generate candidates
            candidates = self.dwa_generator.generate_candidates(
                self.current_state,
                num_candidates=100  # Use random sampling for speed
            )
            
            # Audit feasible set
            feasibility_audit = self.constraint_auditor.audit_feasible_set(
                candidates,
                self.current_state,
                dynamic_obstacles,
                dt=0.1
            )
            
            # Select best candidate (among all, not just feasible)
            # This allows us to see what happens when infeasible actions are selected
            best_candidate, cost_explanation = self.dwa_generator.select_best_candidate(
                candidates,
                self.current_state,
                goal=self.scenario.goal,
                obstacles=dynamic_obstacles,
                weights=self.scenario.cost_weights
            )
            
            # Generate explanation
            explanation = self.explainer.explain_decision(
                best_candidate,
                candidates,
                cost_explanation,
                feasibility_audit,
                self.current_state,
                goal=self.scenario.goal,
                obstacles=dynamic_obstacles
            )
            
            # Convert to RPM and execute
            action_vel = best_candidate['action']
            vx, vy, vz, yaw_rate = action_vel
            rpm = self._velocity_to_rpm(np.array([vx, vy, vz]), yaw_rate)
            
            # Step environment
            obs, reward, terminated, truncated, info = self.env.step(
                rpm.reshape(1, -1)
            )
            self.current_state = self._get_state_from_obs(obs)

            # Slow down for visual inspection (approx real-time)
            if self.real_time and not self.headless:
                time.sleep(1.0 / 240.0)
            
            # Check for collision/near-collision
            position = self.current_state[:3]
            near_collision = self._check_near_collision(position, dynamic_obstacles)
            
            # Prepare logging data
            candidate_summary = {
                'n_generated': len(candidates),
                'generation_method': 'random_sampling'
            }
            
            constraint_audit_dict = {
                'n_total': feasibility_audit.n_total,
                'n_feasible': feasibility_audit.n_feasible,
                'feasible_ratio': feasibility_audit.feasible_ratio,
                'per_constraint_stats': {
                    name: {
                        'violation_ratio': stats['violation_ratio'],
                        'min_slack': stats['min_slack'],
                        'worst_violation': stats['worst_violation']
                    }
                    for name, stats in feasibility_audit.per_constraint_stats.items()
                }
            }
            
            if feasibility_audit.minimal_relaxation:
                constraint_audit_dict['minimal_relaxation'] = feasibility_audit.minimal_relaxation
            
            # Log every tick
            self.logger.log_tick(
                timestep=step,
                state=self.current_state,
                selected_action=best_candidate['action'],
                candidate_summary=candidate_summary,
                constraint_audit=constraint_audit_dict,
                explanation=explanation
            )
            
            # Log events
            event_triggered = False
            
            # Event: No feasible actions
            if feasibility_audit.n_feasible == 0:
                self.logger.log_event(
                    event_type='no_feasible_actions',
                    timestep=step,
                    state=self.current_state,
                    selected_action=best_candidate['action'],
                    candidate_summary=candidate_summary,
                    constraint_audit=constraint_audit_dict,
                    explanation=explanation,
                    additional_info={
                        'minimal_relaxation': feasibility_audit.minimal_relaxation
                    }
                )
                event_triggered = True
            
            # Event: Near collision
            if near_collision:
                self.logger.log_event(
                    event_type='near_collision',
                    timestep=step,
                    state=self.current_state,
                    selected_action=best_candidate['action'],
                    candidate_summary=candidate_summary,
                    constraint_audit=constraint_audit_dict,
                    explanation=explanation,
                    additional_info={
                        'distance_to_nearest_obstacle': min(
                            np.linalg.norm(position - obs_pos)
                            for obs_pos in dynamic_obstacles
                        )
                    }
                )
                event_triggered = True
            
            # Event: Constraint violation in selected action
            selected_audit = next(
                r for r in feasibility_audit.candidate_results
                if r.candidate_id == best_candidate['id']
            )
            if not selected_audit.is_feasible:
                self.logger.log_event(
                    event_type='constraint_violation',
                    timestep=step,
                    state=self.current_state,
                    selected_action=best_candidate['action'],
                    candidate_summary=candidate_summary,
                    constraint_audit=constraint_audit_dict,
                    explanation=explanation,
                    additional_info={
                        'violations': [
                            {
                                'name': v.constraint_name,
                                'violation_amount': v.violation_amount
                            }
                            for v in selected_audit.violations if v.is_violated
                        ]
                    }
                )
                event_triggered = True
            
            # Periodic output
            if step % 20 == 0 or event_triggered:
                print(f"Step {step:4d} | "
                      f"Pos: [{position[0]:5.2f}, {position[1]:5.2f}, {position[2]:5.2f}] | "
                      f"Feasible: {feasibility_audit.n_feasible}/{feasibility_audit.n_total} "
                      f"({feasibility_audit.feasible_ratio:.1%}) | "
                      f"Selected feasible: {selected_audit.is_feasible}")
            
            if terminated or truncated:
                print(f"\nEpisode terminated at step {step + 1}")
                break
        
        # Keep GUI open if requested
        if self.hold and not self.headless:
            print("\\nGUI를 유지 중입니다. 종료하려면 Enter를 누르세요.")
            try:
                input()
            except KeyboardInterrupt:
                pass

        # Close environment
        self.env.close()
        
        # Print summary
        stats = self.logger.get_stats()
        print("\n" + "=" * 80)
        print("Simulation Complete")
        print("=" * 80)
        print(f"Total steps: {step + 1}")
        print(f"Ticks logged: {stats['ticks_logged']}")
        print(f"Events logged: {stats['events_logged']}")
        print(f"Ticks file: {stats['ticks_file']}")
        print(f"Events file: {stats['events_file']}")
        print("=" * 80)


def main():
    """Main entry point"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Run accident analysis scenario')
    parser.add_argument(
        '--scenario',
        type=str,
        default='A',
        choices=['A', 'B', 'C'],
        help='Scenario to run (A: inevitable, B: wrong decision, C: delayed)'
    )
    parser.add_argument(
        '--gui',
        action='store_true',
        default=False,
        help='Enable GUI (visual mode)'
    )
    parser.add_argument(
        '--real-time',
        action='store_true',
        default=False,
        help='Slow down to real-time for visual inspection (GUI only)'
    )
    parser.add_argument(
        '--hold',
        action='store_true',
        default=False,
        help='Keep GUI open after simulation ends (GUI only)'
    )
    parser.add_argument(
        '--demo-motion',
        action='store_true',
        default=False,
        help='Amplify RPM gains for visible motion (GUI only)'
    )
    
    args = parser.parse_args()
    
    rpm_gain_xy = 50.0
    rpm_gain_z = 100.0
    if args.demo_motion:
        rpm_gain_xy = 300.0
        rpm_gain_z = 600.0

    runner = AccidentAnalysisRunner(
        scenario_name=args.scenario,
        headless=not args.gui,
        real_time=args.real_time,
        hold=args.hold,
        rpm_gain_xy=rpm_gain_xy,
        rpm_gain_z=rpm_gain_z
    )
    
    try:
        runner.run()
    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
