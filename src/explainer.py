"""
Decision-Level Explanation: DWA Action Selection Rationale
Explains why a specific action was chosen over alternatives
"""

from typing import Dict, List, Tuple
import numpy as np
from src.dwa import DWACandidateGenerator
from src.constraints import ConstraintAuditor, FeasibleSetAudit


class DecisionExplainer:
    """
    Provides decision-level explanations for DWA action selection
    """
    
    def __init__(
        self,
        dwa_generator: DWACandidateGenerator,
        constraint_auditor: ConstraintAuditor
    ):
        """
        Initialize decision explainer
        
        Args:
            dwa_generator: DWA candidate generator
            constraint_auditor: Constraint auditor for feasibility analysis
        """
        self.dwa_generator = dwa_generator
        self.constraint_auditor = constraint_auditor
    
    def explain_decision(
        self,
        selected_candidate: Dict,
        candidates: List[Dict],
        cost_explanation: Dict,
        feasibility_audit: FeasibleSetAudit,
        current_state: np.ndarray,
        goal: np.ndarray = None,
        obstacles: List[np.ndarray] = None
    ) -> Dict:
        """
        Generate comprehensive explanation for decision
        
        Args:
            selected_candidate: The chosen candidate
            candidates: All candidates considered
            cost_explanation: Cost-based explanation from DWA
            feasibility_audit: Feasibility audit results
            current_state: Current state vector
            goal: Goal position (optional)
            obstacles: Obstacle positions (optional)
        
        Returns:
            Explanation dictionary with:
                - selected_action: The chosen action
                - selection_reason: Why it was chosen
                - cost_ranking: Cost-based ranking
                - feasibility_info: Feasibility information
                - alternative_options: Top alternatives
        """
        selected_id = selected_candidate['id']
        selected_action = selected_candidate['action'].tolist()
        
        # Find selected candidate in audit results
        selected_audit = next(
            r for r in feasibility_audit.candidate_results
            if r.candidate_id == selected_id
        )
        
        # Build explanation
        explanation = {
            'selected_action': selected_action,
            'selected_id': selected_id,
            'selected_cost': cost_explanation['selected_cost'],
            'is_feasible': selected_audit.is_feasible,
            'feasibility_ratio': feasibility_audit.feasible_ratio,
            'top_2_margin': cost_explanation.get('top_2_margin'),
        }
        
        # Selection reason
        if not selected_audit.is_feasible:
            explanation['selection_reason'] = (
                "Selected action violates constraints. "
                "This indicates an inevitable collision or system limitation."
            )
        elif feasibility_audit.feasible_ratio < 0.1:
            explanation['selection_reason'] = (
                f"Selected from very small feasible set ({feasibility_audit.feasible_ratio:.1%}). "
                "Limited options available due to constraints."
            )
        elif cost_explanation['top_2_margin'] is not None and cost_explanation['top_2_margin'] < 0.1:
            explanation['selection_reason'] = (
                f"Selected by narrow margin ({cost_explanation['top_2_margin']:.3f}). "
                "Multiple similar-cost options available."
            )
        else:
            explanation['selection_reason'] = (
                "Selected based on lowest cost among feasible candidates. "
                f"Margin over next best: {cost_explanation['top_2_margin']:.3f}"
            )
        
        # Cost ranking (top 5)
        top_scores = cost_explanation['scores'][:5]
        explanation['cost_ranking'] = [
            {
                'rank': i + 1,
                'candidate_id': score[0],
                'cost': score[1],
                'is_feasible': next(
                    r.is_feasible for r in feasibility_audit.candidate_results
                    if r.candidate_id == score[0]
                )
            }
            for i, score in enumerate(top_scores)
        ]
        
        # Feasibility information
        explanation['feasibility_info'] = {
            'n_total': feasibility_audit.n_total,
            'n_feasible': feasibility_audit.n_feasible,
            'feasible_ratio': feasibility_audit.feasible_ratio,
            'constraint_violations': {
                name: {
                    'violation_ratio': stats['violation_ratio'],
                    'worst_violation': stats['worst_violation']
                }
                for name, stats in feasibility_audit.per_constraint_stats.items()
            }
        }
        
        # Minimal relaxation (if applicable)
        if feasibility_audit.minimal_relaxation:
            explanation['minimal_relaxation'] = feasibility_audit.minimal_relaxation
            explanation['inevitability'] = (
                "No feasible actions available. Collision may be inevitable "
                "unless constraints are relaxed as shown."
            )
        else:
            explanation['inevitability'] = "Feasible actions exist. Collision not inevitable."
        
        # Alternative options (feasible candidates with lowest cost)
        feasible_candidates = [
            (score[0], score[1])
            for score in cost_explanation['scores']
            if next(
                r.is_feasible for r in feasibility_audit.candidate_results
                if r.candidate_id == score[0]
            )
        ]
        
        if feasible_candidates:
            explanation['best_feasible_alternatives'] = [
                {
                    'candidate_id': cand_id,
                    'cost': cost,
                    'action': next(
                        c['action'].tolist() for c in candidates
                        if c['id'] == cand_id
                    )
                }
                for cand_id, cost in feasible_candidates[:3]
            ]
        
        # Score breakdown for selected candidate
        explanation['score_breakdown'] = self._compute_score_breakdown(
            selected_candidate, current_state, goal, obstacles
        )
        
        return explanation
    
    def _compute_score_breakdown(
        self,
        candidate: Dict,
        current_state: np.ndarray,
        goal: np.ndarray,
        obstacles: List[np.ndarray]
    ) -> Dict:
        """
        Break down cost components for selected candidate
        
        Returns:
            Dictionary with cost component values
        """
        action = candidate['action']
        vx, vy, vz, yaw_rate = action
        target_velocity = np.array([vx, vy, vz])
        
        dt = 0.1
        current_pos = current_state[:3]
        current_vel = current_state[3:6]
        predicted_pos = current_pos + target_velocity * dt
        
        breakdown = {}
        
        # Goal distance component
        if goal is not None:
            goal_dist = np.linalg.norm(predicted_pos - goal)
            breakdown['goal_distance'] = goal_dist
        
        # Obstacle proximity component
        if obstacles:
            min_obstacle_dist = min(
                np.linalg.norm(predicted_pos - obs_pos)
                for obs_pos in obstacles
            )
            breakdown['obstacle_proximity'] = min_obstacle_dist
            if min_obstacle_dist < 1.0:
                breakdown['obstacle_penalty'] = np.exp(-min_obstacle_dist)
        
        # Velocity magnitude
        velocity_mag = np.linalg.norm(target_velocity)
        breakdown['velocity_magnitude'] = velocity_mag
        
        # Yaw rate
        breakdown['yaw_rate_magnitude'] = abs(yaw_rate)
        
        return breakdown
