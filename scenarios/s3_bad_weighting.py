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
            "N_feasible stays > 0 at trigger; Top-K includes a safer feasible detour; "
            "the chosen action wins mainly by the goal_distance term, leading to near-miss/collision."
        ),
        description=(
            "A safe detour exists, but cost weights over-emphasize goal progress so the planner "
            "selects a risky near-straight trajectory despite feasible safer alternatives."
        ),
        seed=0,
        max_steps=300,
        candidate_count=200,
        v_min=0.0,
        v_max=2.0,
        yaw_rate_min=-1.0,
        yaw_rate_max=1.0,
        initial_position=np.array([0.0, 0.0, 1.0]),
        initial_velocity=np.array([1.2, 0.0, 0.0]),
        goal=np.array([5.0, 0.0, 1.0]),
        obstacles=[
            np.array([2.2, 0.05, 1.0]),
        ],
        obstacle_shape="sphere",
        obstacle_size={"radius": 0.35},
        constraint_config=ConstraintConfig(
            min_obstacle_distance=0.6,
            max_acceleration=1.8,
            max_yaw_rate=0.9,
            max_velocity=2.5,
            min_altitude=0.8,
            max_altitude=1.5,
        ),
        cost_weights={
            "goal_distance": 10.0,
            "obstacle": 1.0,
            "velocity": 0.1,
            "yaw_rate": 0.4,
        },
    )
