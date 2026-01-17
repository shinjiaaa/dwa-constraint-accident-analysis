"""
Reproducible Scenarios for Accident Post-Hoc Analysis

Three scenarios with clear ground-truth causes:
A) Feasible = empty (inevitable collision)
B) Feasible exists but wrong decision due to cost weighting
C) Timing issue (delayed response)
"""

import numpy as np
from typing import List, Tuple, Dict
from dataclasses import dataclass
from src.constraints import ConstraintConfig


@dataclass
class ScenarioConfig:
    """Configuration for a scenario"""
    name: str
    description: str
    initial_position: np.ndarray
    initial_velocity: np.ndarray
    goal: np.ndarray
    obstacles: List[np.ndarray]
    constraint_config: ConstraintConfig
    cost_weights: Dict[str, float]
    max_steps: int
    ground_truth_cause: str


class ScenarioFactory:
    """Factory for creating reproducible scenarios"""
    
    @staticmethod
    def scenario_a_inevitable_collision() -> ScenarioConfig:
        """
        Scenario A: Feasible = empty (inevitable collision)
        
        Setup:
        - Drone starts at (0, 0, 1)
        - Goal at (5, 0, 1)
        - Obstacle at (2.5, 0, 1) - directly in path
        - Very tight constraints that make all paths infeasible
        
        Ground truth: Collision is inevitable due to strict constraints
        and obstacle placement. No feasible actions exist.
        """
        return ScenarioConfig(
            name="A_inevitable_collision",
            description=(
                "Drone must pass through narrow gap but constraints "
                "make all actions infeasible. Collision is inevitable."
            ),
            initial_position=np.array([0.0, 0.0, 1.0]),
            initial_velocity=np.array([0.0, 0.0, 0.0]),
            goal=np.array([5.0, 0.0, 1.0]),
            obstacles=[
                np.array([2.5, 0.0, 1.0]),  # Directly in path
                np.array([2.5, 0.5, 1.0]),  # Blocking left side
                np.array([2.5, -0.5, 1.0]),  # Blocking right side
            ],
            constraint_config=ConstraintConfig(
                min_obstacle_distance=0.8,  # Large safety margin
                max_acceleration=2.0,  # Low acceleration limit
                max_yaw_rate=0.5,  # Low yaw rate
                max_velocity=1.0,  # Low speed limit
                min_altitude=0.5,
                max_altitude=2.0
            ),
            cost_weights={
                'goal_distance': 1.0,
                'obstacle': 5.0,
                'velocity': 0.1,
                'yaw_rate': 0.5
            },
            max_steps=100,
            ground_truth_cause=(
                "Inevitable collision: Obstacle directly in path with "
                "strict constraints (min_obstacle_distance=0.8m, max_velocity=1.0m/s). "
                "All candidate actions violate constraints. "
                "Minimal relaxation analysis should show required constraint relaxation."
            )
        )
    
    @staticmethod
    def scenario_b_wrong_decision() -> ScenarioConfig:
        """
        Scenario B: Feasible exists but wrong decision due to cost weighting
        
        Setup:
        - Drone starts at (0, 0, 1)
        - Goal at (5, 0, 1)
        - Obstacle slightly offset at (2.5, 0.3, 1)
        - Feasible path exists (go left around obstacle)
        - But cost function heavily weights goal_distance, ignoring obstacle
        
        Ground truth: Feasible safe path exists, but cost weighting causes
        selection of unsafe action that violates constraints.
        """
        return ScenarioConfig(
            name="B_wrong_decision",
            description=(
                "Feasible safe path exists (left around obstacle), "
                "but cost function weights goal_distance too highly, "
                "selecting unsafe direct path."
            ),
            initial_position=np.array([0.0, 0.0, 1.0]),
            initial_velocity=np.array([0.0, 0.0, 0.0]),
            goal=np.array([5.0, 0.0, 1.0]),
            obstacles=[
                np.array([2.5, 0.3, 1.0]),  # Slightly offset, safe path exists
            ],
            constraint_config=ConstraintConfig(
                min_obstacle_distance=0.5,
                max_acceleration=5.0,
                max_yaw_rate=1.5,
                max_velocity=3.0,
                min_altitude=0.5,
                max_altitude=2.0
            ),
            cost_weights={
                'goal_distance': 10.0,  # Very high weight on goal
                'obstacle': 1.0,  # Low weight on obstacle avoidance
                'velocity': 0.1,
                'yaw_rate': 0.5
            },
            max_steps=150,
            ground_truth_cause=(
                "Wrong decision: Feasible safe path exists (left around obstacle), "
                "but cost function heavily weights goal_distance (10.0) vs obstacle (1.0). "
                "DWA selects direct path to goal, violating obstacle distance constraint. "
                "Decision-level explanation should show cost breakdown and feasible alternatives."
            )
        )
    
    @staticmethod
    def scenario_c_delayed_response() -> ScenarioConfig:
        """
        Scenario C: Timing issue (delayed response)
        
        Setup:
        - Drone starts moving fast toward goal
        - Obstacle appears late (at t=30 steps)
        - Drone has insufficient time/distance to avoid
        
        Ground truth: Feasible actions exist at each step, but delayed
        obstacle detection causes insufficient reaction time.
        """
        return ScenarioConfig(
            name="C_delayed_response",
            description=(
                "Drone moving fast. Obstacle appears late. "
                "Feasible actions exist individually, but cumulative effect "
                "of delayed detection leads to collision."
            ),
            initial_position=np.array([0.0, 0.0, 1.0]),
            initial_velocity=np.array([1.5, 0.0, 0.0]),  # Moving fast
            goal=np.array([10.0, 0.0, 1.0]),
            obstacles=[
                # Obstacle appears at step 30 (will be added dynamically)
                np.array([4.5, 0.0, 1.0]),  # Close to path
            ],
            constraint_config=ConstraintConfig(
                min_obstacle_distance=0.5,
                max_acceleration=3.0,  # Moderate acceleration
                max_yaw_rate=1.0,
                max_velocity=2.0,
                min_altitude=0.5,
                max_altitude=2.0
            ),
            cost_weights={
                'goal_distance': 1.0,
                'obstacle': 5.0,
                'velocity': 0.1,
                'yaw_rate': 0.5
            },
            max_steps=200,
            ground_truth_cause=(
                "Delayed response: Drone moving at 1.5 m/s toward goal. "
                "Obstacle detection/reactivation delayed. "
                "At each step, feasible actions exist, but insufficient time "
                "to safely avoid due to high velocity and limited acceleration. "
                "Temporal analysis needed to detect delayed response pattern."
            )
        )
    
    @staticmethod
    def get_scenario(scenario_name: str) -> ScenarioConfig:
        """Get scenario by name"""
        scenarios = {
            'A': ScenarioFactory.scenario_a_inevitable_collision,
            'B': ScenarioFactory.scenario_b_wrong_decision,
            'C': ScenarioFactory.scenario_c_delayed_response,
        }
        
        if scenario_name.upper() not in scenarios:
            raise ValueError(
                f"Unknown scenario: {scenario_name}. "
                f"Available: {list(scenarios.keys())}"
            )
        
        return scenarios[scenario_name.upper()]()
    
    @staticmethod
    def list_scenarios() -> List[str]:
        """List available scenario names"""
        return ['A', 'B', 'C']
