"""
Dynamic Window Approach (DWA) - Candidate Action Generation
Simple, deterministic candidate sampler for drone control
"""

import numpy as np
from typing import List, Tuple, Dict


class DWACandidateGenerator:
    """
    DWA-style candidate action generator for drone control
    
    For quadrotor drones, we work in velocity space:
    - Linear velocity (vx, vy, vz)
    - Yaw rate (omega_z)
    
    In practice, these are converted to RPM commands by the control system.
    """
    
    def __init__(
        self,
        v_min: float = -2.0,
        v_max: float = 2.0,
        yaw_rate_min: float = -1.0,
        yaw_rate_max: float = 1.0,
        v_resolution: int = 5,
        yaw_rate_resolution: int = 5,
        seed: int = 42
    ):
        """
        Initialize DWA candidate generator
        
        Args:
            v_min: Minimum velocity magnitude (m/s)
            v_max: Maximum velocity magnitude (m/s)
            yaw_rate_min: Minimum yaw rate (rad/s)
            yaw_rate_max: Maximum yaw rate (rad/s)
            v_resolution: Number of velocity samples per axis
            yaw_rate_resolution: Number of yaw rate samples
            seed: Random seed for reproducibility
        """
        self.v_min = v_min
        self.v_max = v_max
        self.yaw_rate_min = yaw_rate_min
        self.yaw_rate_max = yaw_rate_max
        self.v_resolution = v_resolution
        self.yaw_rate_resolution = yaw_rate_resolution
        
        np.random.seed(seed)
        self._rng = np.random.default_rng(seed)
    
    def generate_candidates(
        self,
        current_state: np.ndarray,
        num_candidates: int = None
    ) -> List[Dict]:
        """
        Generate candidate actions using DWA-style sampling
        
        Args:
            current_state: Current state vector [x, y, z, vx, vy, vz, ...]
            num_candidates: Number of candidates to generate (if None, uses grid)
        
        Returns:
            List of candidate dictionaries with keys:
                - action: velocity command [vx, vy, vz, yaw_rate]
                - id: candidate identifier
        """
        candidates = []
        
        if num_candidates is None:
            # Grid-based sampling (deterministic)
            candidates = self._generate_grid_candidates()
        else:
            # Random sampling (but deterministic with seed)
            candidates = self._generate_random_candidates(num_candidates)
        
        return candidates
    
    def _generate_grid_candidates(self) -> List[Dict]:
        """Generate candidates using regular grid sampling"""
        candidates = []
        candidate_id = 0
        
        # Sample velocity components
        vx_values = np.linspace(self.v_min, self.v_max, self.v_resolution)
        vy_values = np.linspace(self.v_min, self.v_max, self.v_resolution)
        vz_values = np.linspace(-0.5, 0.5, 3)  # Limited vertical velocity
        yaw_rate_values = np.linspace(
            self.yaw_rate_min, 
            self.yaw_rate_max, 
            self.yaw_rate_resolution
        )
        
        # Generate all combinations
        for vx in vx_values:
            for vy in vy_values:
                for vz in vz_values:
                    for yaw_rate in yaw_rate_values:
                        action = np.array([vx, vy, vz, yaw_rate])
                        candidates.append({
                            'action': action,
                            'id': candidate_id
                        })
                        candidate_id += 1
        
        return candidates
    
    def _generate_random_candidates(self, num_candidates: int) -> List[Dict]:
        """Generate candidates using random sampling (deterministic with seed)"""
        candidates = []
        
        for i in range(num_candidates):
            vx = self._rng.uniform(self.v_min, self.v_max)
            vy = self._rng.uniform(self.v_min, self.v_max)
            vz = self._rng.uniform(-0.5, 0.5)
            yaw_rate = self._rng.uniform(self.yaw_rate_min, self.yaw_rate_max)
            
            action = np.array([vx, vy, vz, yaw_rate])
            candidates.append({
                'action': action,
                'id': i
            })
        
        return candidates
    
    def compute_cost(
        self,
        candidate: Dict,
        current_state: np.ndarray,
        goal: np.ndarray = None,
        obstacles: List[np.ndarray] = None,
        weights: Dict[str, float] = None
    ) -> float:
        """
        Compute cost for a candidate action
        
        Simple cost function with multiple components:
        - Goal distance (if goal provided)
        - Obstacle proximity (if obstacles provided)
        - Velocity magnitude (prefer moderate speeds)
        - Yaw rate (prefer small yaw rates)
        
        Args:
            candidate: Candidate dictionary with 'action' key
            current_state: Current state [x, y, z, vx, vy, vz, ...]
            goal: Goal position [x, y, z] (optional)
            obstacles: List of obstacle positions (optional)
            weights: Cost component weights
        
        Returns:
            Total cost (lower is better)
        """
        if weights is None:
            weights = {
                'goal_distance': 1.0,
                'obstacle': 5.0,
                'velocity': 0.1,
                'yaw_rate': 0.5
            }
        
        action = candidate['action']
        vx, vy, vz, yaw_rate = action
        
        cost = 0.0
        
        # Predict next position (simple kinematic model)
        dt = 0.1  # Assume 100ms control period
        current_pos = current_state[:3]
        current_vel = current_state[3:6]
        
        # Predicted position after action
        predicted_vel = np.array([vx, vy, vz])
        predicted_pos = current_pos + predicted_vel * dt
        
        # Goal distance cost
        if goal is not None:
            goal_dist = np.linalg.norm(predicted_pos - goal)
            cost += weights['goal_distance'] * goal_dist
        
        # Obstacle proximity cost
        if obstacles is not None:
            min_obstacle_dist = float('inf')
            for obs_pos in obstacles:
                dist = np.linalg.norm(predicted_pos - obs_pos)
                min_obstacle_dist = min(min_obstacle_dist, dist)
            
            # Exponential penalty for close obstacles
            if min_obstacle_dist < 1.0:
                cost += weights['obstacle'] * np.exp(-min_obstacle_dist)
        
        # Velocity magnitude cost (prefer moderate speeds)
        velocity_mag = np.linalg.norm(predicted_vel)
        cost += weights['velocity'] * velocity_mag ** 2
        
        # Yaw rate cost (prefer small yaw rates)
        cost += weights['yaw_rate'] * abs(yaw_rate) ** 2
        
        return cost
    
    def select_best_candidate(
        self,
        candidates: List[Dict],
        current_state: np.ndarray,
        goal: np.ndarray = None,
        obstacles: List[np.ndarray] = None,
        weights: Dict[str, float] = None
    ) -> Tuple[Dict, Dict]:
        """
        Select best candidate based on cost function
        
        Returns:
            (best_candidate, explanation_dict)
            explanation_dict contains:
                - scores: list of (candidate_id, cost) tuples sorted by cost
                - top_2_margin: cost difference between top-1 and top-2
                - selected_id: ID of selected candidate
        """
        # Compute costs for all candidates
        scores = []
        for cand in candidates:
            cost = self.compute_cost(cand, current_state, goal, obstacles, weights)
            scores.append((cand['id'], cost))
        
        # Sort by cost (lower is better)
        scores.sort(key=lambda x: x[1])
        
        # Get best candidate
        best_id = scores[0][0]
        best_candidate = next(c for c in candidates if c['id'] == best_id)
        
        # Compute margin
        top_2_margin = None
        if len(scores) > 1:
            top_2_margin = scores[1][1] - scores[0][1]
        
        explanation = {
            'scores': scores,  # List of (id, cost) sorted by cost
            'top_2_margin': top_2_margin,
            'selected_id': best_id,
            'selected_cost': scores[0][1]
        }
        
        return best_candidate, explanation
