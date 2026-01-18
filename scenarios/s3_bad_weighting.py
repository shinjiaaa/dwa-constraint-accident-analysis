"""
S3: Feasible exists but wrong decision due to cost weighting (Feasible set != empty).
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


def build_s3() -> ScenarioConfig:
    return ScenarioConfig(
        name="s3",
        gt_label="BAD_DECISION_WEIGHTING",
        expected_xai=(
            "At trigger, N_feasible > 0 and Top-K contains a safer feasible detour. "
            "However, the selected action wins due to the goal term dominance, leading to collision/near-miss."
        ),
        description=(
            "A safe detour exists, but cost weights over-emphasize goal progress; the planner selects "
            "a risky near-straight trajectory and collides/near-misses despite feasible safer options."
        ),
        seed=0,
        max_steps=300,
        candidate_count=200,
        v_min=0.0,
        v_max=2.0,
        yaw_rate_min=-0.8,
        yaw_rate_max=0.8,
        initial_position=np.array([0.0, 0.0, 1.0]),
        initial_velocity=np.array([1.0, 0.0, 0.0]),  # Fast enough to create urgency
        goal=np.array([5.0, 0.0, 1.0]),
        obstacles=[np.array([1.5, 0.4, 1.0])],  # Start more offset, will move aggressively to block
        obstacle_shape="sphere",
        obstacle_size={"radius": 0.3},  # Larger for more obvious collision
        constraint_config=ConstraintConfig(
            min_obstacle_distance=0.4,  # Very relaxed to ensure feasible alternatives exist
            max_acceleration=4.0,  # Very relaxed
            max_yaw_rate=1.5,  # Very relaxed
            max_velocity=2.5,
            min_altitude=0.7,
            max_altitude=1.6,
        ),
        cost_weights={
            "goal_distance": 20.0,  # EXTREMELY high to force straight path selection
            "obstacle": 0.5,  # Very low to allow near-miss
            "velocity": 0.1,
            "yaw_rate": 0.3,
        },
    )
