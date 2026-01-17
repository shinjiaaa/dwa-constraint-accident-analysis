"""
Constraint-Based XAI: Feasibility Analysis and Minimal Relaxation
Core module for constraint definition, violation detection, and counterfactual analysis
"""

import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass


@dataclass
class ConstraintConfig:
    """Configuration for safety constraints"""
    min_obstacle_distance: float = 0.5  # meters
    max_acceleration: float = 5.0  # m/s^2
    max_yaw_rate: float = 1.5  # rad/s
    max_velocity: float = 3.0  # m/s
    min_altitude: float = 0.1  # meters (above ground)
    max_altitude: float = 10.0  # meters


@dataclass
class ConstraintViolation:
    """Record of a constraint violation"""
    constraint_name: str
    violation_amount: float  # Positive value = violation, negative = slack
    threshold: float
    is_violated: bool


@dataclass
class CandidateAuditResult:
    """Audit result for a single candidate"""
    candidate_id: int
    is_feasible: bool
    violations: List[ConstraintViolation]
    total_violation_score: float  # Sum of all violations


@dataclass
class FeasibleSetAudit:
    """Results of auditing all candidates"""
    n_total: int
    n_feasible: int
    feasible_ratio: float
    per_constraint_stats: Dict[str, Dict]  # violation_ratio, min_slack, worst_violation
    candidate_results: List[CandidateAuditResult]
    minimal_relaxation: Optional[Dict[str, float]]  # None if feasible set is non-empty


class ConstraintAuditor:
    """
    Audits candidate actions against safety constraints
    Computes feasibility, violations, and minimal relaxation
    """
    
    def __init__(self, config: ConstraintConfig = None):
        """
        Initialize constraint auditor
        
        Args:
            config: Constraint configuration (uses defaults if None)
        """
        self.config = config or ConstraintConfig()
    
    def check_obstacle_distance(
        self,
        predicted_position: np.ndarray,
        obstacles: List[np.ndarray]
    ) -> ConstraintViolation:
        """
        Check minimum obstacle distance constraint
        
        Args:
            predicted_position: Predicted position after action [x, y, z]
            obstacles: List of obstacle positions [[x, y, z], ...]
        
        Returns:
            ConstraintViolation object
        """
        if not obstacles:
            # No obstacles = always satisfied
            return ConstraintViolation(
                constraint_name="min_obstacle_distance",
                violation_amount=float('inf'),  # Infinite slack
                threshold=self.config.min_obstacle_distance,
                is_violated=False
            )
        
        # Find minimum distance to any obstacle
        min_dist = float('inf')
        for obs_pos in obstacles:
            dist = np.linalg.norm(predicted_position - obs_pos)
            min_dist = min(min_dist, dist)
        
        # Slack = distance - threshold (positive = satisfied, negative = violated)
        slack = min_dist - self.config.min_obstacle_distance
        violation_amount = -slack  # Positive = violation
        
        return ConstraintViolation(
            constraint_name="min_obstacle_distance",
            violation_amount=violation_amount,
            threshold=self.config.min_obstacle_distance,
            is_violated=(slack < 0)
        )
    
    def check_acceleration(
        self,
        current_velocity: np.ndarray,
        target_velocity: np.ndarray,
        dt: float = 0.1
    ) -> ConstraintViolation:
        """
        Check maximum acceleration constraint
        
        Args:
            current_velocity: Current velocity [vx, vy, vz]
            target_velocity: Target velocity from action [vx, vy, vz]
            dt: Time step (seconds)
        
        Returns:
            ConstraintViolation object
        """
        # Compute required acceleration
        velocity_change = target_velocity - current_velocity
        acceleration = np.linalg.norm(velocity_change) / dt
        
        # Slack = threshold - acceleration (positive = satisfied, negative = violated)
        slack = self.config.max_acceleration - acceleration
        violation_amount = -slack  # Positive = violation
        
        return ConstraintViolation(
            constraint_name="max_acceleration",
            violation_amount=violation_amount,
            threshold=self.config.max_acceleration,
            is_violated=(slack < 0)
        )
    
    def check_yaw_rate(
        self,
        yaw_rate: float
    ) -> ConstraintViolation:
        """
        Check maximum yaw rate constraint
        
        Args:
            yaw_rate: Yaw rate command (rad/s)
        
        Returns:
            ConstraintViolation object
        """
        abs_yaw_rate = abs(yaw_rate)
        slack = self.config.max_yaw_rate - abs_yaw_rate
        violation_amount = -slack  # Positive = violation
        
        return ConstraintViolation(
            constraint_name="max_yaw_rate",
            violation_amount=violation_amount,
            threshold=self.config.max_yaw_rate,
            is_violated=(slack < 0)
        )
    
    def check_velocity(
        self,
        velocity: np.ndarray
    ) -> ConstraintViolation:
        """
        Check maximum velocity constraint
        
        Args:
            velocity: Velocity vector [vx, vy, vz]
        
        Returns:
            ConstraintViolation object
        """
        velocity_mag = np.linalg.norm(velocity)
        slack = self.config.max_velocity - velocity_mag
        violation_amount = -slack  # Positive = violation
        
        return ConstraintViolation(
            constraint_name="max_velocity",
            violation_amount=violation_amount,
            threshold=self.config.max_velocity,
            is_violated=(slack < 0)
        )
    
    def check_altitude(
        self,
        position: np.ndarray
    ) -> ConstraintViolation:
        """
        Check altitude bounds constraint
        
        Args:
            position: Position [x, y, z] (z is altitude)
        
        Returns:
            ConstraintViolation object (worst violation if both bounds exist)
        """
        z = position[2]
        
        # Check lower bound
        min_alt_slack = z - self.config.min_altitude
        min_alt_violation = -min_alt_slack if min_alt_slack < 0 else 0.0
        
        # Check upper bound
        max_alt_slack = self.config.max_altitude - z
        max_alt_violation = -max_alt_slack if max_alt_slack < 0 else 0.0
        
        # Worst violation
        worst_violation = max(min_alt_violation, max_alt_violation)
        is_violated = (min_alt_slack < 0) or (max_alt_slack < 0)
        
        return ConstraintViolation(
            constraint_name="altitude_bounds",
            violation_amount=worst_violation,
            threshold=0.0,  # Composite constraint
            is_violated=is_violated
        )
    
    def audit_candidate(
        self,
        candidate: Dict,
        current_state: np.ndarray,
        obstacles: List[np.ndarray],
        dt: float = 0.1
    ) -> CandidateAuditResult:
        """
        Audit a single candidate against all constraints
        
        Args:
            candidate: Candidate dictionary with 'action' key [vx, vy, vz, yaw_rate]
            current_state: Current state [x, y, z, vx, vy, vz, ...]
            obstacles: List of obstacle positions
            dt: Time step for prediction
        
        Returns:
            CandidateAuditResult
        """
        action = candidate['action']
        vx, vy, vz, yaw_rate = action
        target_velocity = np.array([vx, vy, vz])
        
        current_pos = current_state[:3]
        current_vel = current_state[3:6]
        
        # Predict next position
        predicted_pos = current_pos + target_velocity * dt
        
        # Check all constraints
        violations = []
        
        violations.append(self.check_obstacle_distance(predicted_pos, obstacles))
        violations.append(self.check_acceleration(current_vel, target_velocity, dt))
        violations.append(self.check_yaw_rate(yaw_rate))
        violations.append(self.check_velocity(target_velocity))
        violations.append(self.check_altitude(predicted_pos))
        
        # Determine feasibility
        is_feasible = all(not v.is_violated for v in violations)
        
        # Total violation score (sum of all violations)
        total_violation_score = sum(
            max(0, v.violation_amount) for v in violations
        )
        
        return CandidateAuditResult(
            candidate_id=candidate['id'],
            is_feasible=is_feasible,
            violations=violations,
            total_violation_score=total_violation_score
        )
    
    def audit_feasible_set(
        self,
        candidates: List[Dict],
        current_state: np.ndarray,
        obstacles: List[np.ndarray],
        dt: float = 0.1
    ) -> FeasibleSetAudit:
        """
        Audit all candidates and compute feasible set statistics
        
        Args:
            candidates: List of candidate dictionaries
            current_state: Current state vector
            obstacles: List of obstacle positions
            dt: Time step for prediction
        
        Returns:
            FeasibleSetAudit with complete statistics
        """
        n_total = len(candidates)
        
        # Audit all candidates
        candidate_results = []
        for cand in candidates:
            result = self.audit_candidate(cand, current_state, obstacles, dt)
            candidate_results.append(result)
        
        # Count feasible
        n_feasible = sum(1 for r in candidate_results if r.is_feasible)
        feasible_ratio = n_feasible / n_total if n_total > 0 else 0.0
        
        # Per-constraint statistics
        constraint_names = set()
        for result in candidate_results:
            for v in result.violations:
                constraint_names.add(v.constraint_name)
        
        per_constraint_stats = {}
        for constraint_name in constraint_names:
            # Collect violations for this constraint
            constraint_violations = []
            for result in candidate_results:
                for v in result.violations:
                    if v.constraint_name == constraint_name:
                        constraint_violations.append(v)
            
            # Compute statistics
            n_violated = sum(1 for v in constraint_violations if v.is_violated)
            violation_ratio = n_violated / n_total if n_total > 0 else 0.0
            
            # Min slack (most negative = worst violation)
            min_slack = min(-v.violation_amount for v in constraint_violations)
            
            # Worst violation (most positive = worst)
            worst_violation = max(
                max(0, v.violation_amount) for v in constraint_violations
            )
            
            per_constraint_stats[constraint_name] = {
                'violation_ratio': violation_ratio,
                'min_slack': min_slack,
                'worst_violation': worst_violation
            }
        
        # Minimal relaxation (if no feasible candidates)
        minimal_relaxation = None
        if n_feasible == 0:
            minimal_relaxation = self._compute_minimal_relaxation(
                candidate_results, constraint_names
            )
        
        return FeasibleSetAudit(
            n_total=n_total,
            n_feasible=n_feasible,
            feasible_ratio=feasible_ratio,
            per_constraint_stats=per_constraint_stats,
            candidate_results=candidate_results,
            minimal_relaxation=minimal_relaxation
        )
    
    def _compute_minimal_relaxation(
        self,
        candidate_results: List[CandidateAuditResult],
        constraint_names: set
    ) -> Dict[str, float]:
        """
        Compute minimal relaxation needed to make at least one candidate feasible
        
        This is a counterfactual analysis: "How much must each constraint be relaxed
        to allow at least one feasible action?"
        
        Args:
            candidate_results: List of audit results
            constraint_names: Set of constraint names
        
        Returns:
            Dictionary mapping constraint_name -> required relaxation delta
        """
        relaxation = {}
        
        for constraint_name in constraint_names:
            # Find the minimum violation amount across all candidates for this constraint
            min_violation = float('inf')
            
            for result in candidate_results:
                for v in result.violations:
                    if v.constraint_name == constraint_name and v.is_violated:
                        min_violation = min(min_violation, v.violation_amount)
            
            # If we found violations, relaxation = violation amount
            # This is the minimum amount we need to relax to make the least-violated
            # candidate feasible
            if min_violation < float('inf'):
                relaxation[constraint_name] = min_violation
            else:
                relaxation[constraint_name] = 0.0
        
        return relaxation
