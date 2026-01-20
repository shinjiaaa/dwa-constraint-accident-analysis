"""
S1: Inevitable collision due to dynamics (Feasible set = empty).

This scenario demonstrates the "inevitability" (불가피성) category:
- Feasible set collapses to empty (N_feasible == 0) before collision
- Constraint saturation prevents safe alternatives
- Used to test the system's ability to identify inevitable accidents
  through feasible set auditing and minimal relaxation analysis
"""

from dataclasses import dataclass
import numpy as np
from src.constraints import ConstraintConfig


@dataclass
class ScenarioConfig:
    name: str
    gt_label: str
    expected_xai: str
    description: str
    seed: int
    max_steps: int
    candidate_count: int
    v_min: float
    v_max: float
    yaw_rate_min: float
    yaw_rate_max: float
    initial_position: np.ndarray
    initial_velocity: np.ndarray
    goal: np.ndarray
    obstacles: list
    obstacle_shape: str
    obstacle_size: dict
    constraint_config: ConstraintConfig
    cost_weights: dict


def build_s1() -> ScenarioConfig:
    return ScenarioConfig(
        name="s1",
        gt_label="INEVITABLE_DYNAMICS",
        expected_xai=(
            "N_feasible collapses to 0 before collision; accel constraint dominates; "
            "minimal relaxation shows large Δ_accel."
        ),
        description=(
            "High forward speed toward a close obstacle with strict accel/brake limits. "
            "Feasible set collapses to empty before collision."
        ),
        seed=0,
        max_steps=240,
        candidate_count=200,
        v_min=-2.0,
        v_max=2.0,
        yaw_rate_min=-1.0,
        yaw_rate_max=1.0,
        initial_position=np.array([0.0, 0.0, 1.0]),
        initial_velocity=np.array([2.0, 0.0, 0.0]),
        goal=np.array([6.0, 0.0, 1.0]),
        obstacles=[
            np.array([2.0, 0.0, 1.0]),
        ],
        # Large wall to make failure visually obvious
        obstacle_shape="wall",
        obstacle_size={"half_extents": [0.05, 1.2, 0.6]},
        constraint_config=ConstraintConfig(
            min_obstacle_distance=0.6,
            max_acceleration=1.0,
            max_yaw_rate=0.6,
            max_velocity=2.5,
            min_altitude=0.8,
            max_altitude=1.2,
        ),
        cost_weights={
            "goal_distance": 1.0,
            "obstacle": 6.0,
            "velocity": 0.1,
            "yaw_rate": 0.5,
        },
    )
