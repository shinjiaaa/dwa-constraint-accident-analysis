"""
S2: Feasible exists but wrong decision due to cost structure (Feasible set != empty).

This scenario demonstrates the "choice error" (선택오류) category:
- Feasible alternatives exist (N_feasible > 0)
- But the planner selects a risky action due to cost structure/weighting
- Used to test the system's ability to detect safer feasible alternatives
  and explain why the selected action was chosen (decision rationale)
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

def build_s2() -> ScenarioConfig:
    return ScenarioConfig(
        name="s2",
        gt_label="BAD_DECISION",
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

        # 샘플 충분히
        candidate_count=1200,

        v_min=0.0,
        v_max=2.2,

        # yaw 후보 다양성
        yaw_rate_min=-2.2,
        yaw_rate_max=2.2,

        initial_position=np.array([0.0, 0.0, 1.0]),
        initial_velocity=np.array([1.1, 0.0, 0.0]),

        goal=np.array([6.0, 0.0, 1.0]),

        # ✅ 핵심: "정면 위험 + 옆으로 넉넉한 우회로"를 강제로 만들기 위해 2개 배치
        # - 중앙 큰 장애물: 직진은 위험하게
        # - 위쪽 작은 장애물: 위쪽 우회로는 덜 안전(또는 좁게) 만들어서
        #   "아래쪽 우회로가 더 안전"한 feasible 대안이 항상 생기게
        obstacles=[
            np.array([3.0, 0.0, 1.0]),   # 중앙(직진 위험 유도)
            np.array([3.0, 0.85, 1.0]),  # 위쪽(위 detour를 덜 안전하게)
        ],
        obstacle_shape="sphere",

        # 여기서 size가 dict 하나면 "모든 장애물 동일 radius"로 적용될 가능성이 커서,
        # 중앙을 크게 할 거면 radius를 크게 두는 게 안전함.
        # (만약 장애물별 radius를 지원하면, 중앙만 크게/위쪽만 작게를 권장)
        obstacle_size={"radius": 0.55},

        constraint_config=ConstraintConfig(
            # ✅ feasibility 붕괴 방지: margin 거의 0으로.
            # (너의 feasibility가 dist(center)-r 방식이면 이 값이 조금만 커도 바로 1개로 붕괴함)
            min_obstacle_distance=0.0,

            max_acceleration=10.0,
            max_yaw_rate=3.0,
            max_velocity=3.0,
            min_altitude=0.7,
            max_altitude=1.6,
        ),

        cost_weights={
            # ✅ goal 우세로 위험 직진을 고르게
            "goal_distance": 40.0,

            # ✅ obstacle 비용 거의 무시 -> near-miss/충돌 성향
            "obstacle": 0.03,

            # ✅ detour가 yaw_rate 때문에 밀리지 않게 (매우 작게)
            "yaw_rate": 0.005,

            # ✅ velocity도 거의 영향 없게
            "velocity": 0.01,
        },
    )

