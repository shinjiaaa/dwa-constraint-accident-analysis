"""
JSONL Logging System for Post-Hoc Accident Analysis
Logs simulation ticks and critical events
"""

import json
import os
from datetime import datetime
from typing import Dict, List, Optional
import numpy as np


class JSONLLogger:
    """
    JSONL (JSON Lines) logger for simulation data
    Logs every timestep (ticks.jsonl) and critical events (events.jsonl)
    """
    
    def __init__(self, log_dir: str = "logs"):
        """
        Initialize logger
        
        Args:
            log_dir: Directory for log files
        """
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        
        # Create log files with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.ticks_file = os.path.join(log_dir, f"ticks_{timestamp}.jsonl")
        self.events_file = os.path.join(log_dir, f"events_{timestamp}.jsonl")
        
        self.tick_count = 0
        self.event_count = 0
    
    def log_tick(
        self,
        timestep: int,
        state: np.ndarray,
        selected_action: np.ndarray,
        candidate_summary: Dict,
        constraint_audit: Dict,
        explanation: Dict
    ):
        """
        Log every simulation timestep
        
        Args:
            timestep: Current simulation step
            state: Current state vector
            selected_action: Selected action [vx, vy, vz, yaw_rate]
            candidate_summary: Summary of candidate generation
            constraint_audit: Constraint audit results
            explanation: Decision explanation
        """
        log_entry = {
            'timestep': timestep,
            'tick_id': self.tick_count,
            'timestamp': datetime.now().isoformat(),
            'state': {
                'position': state[:3].tolist(),
                'velocity': state[3:6].tolist(),
            },
            'selected_action': selected_action.tolist() if isinstance(selected_action, np.ndarray) else selected_action,
            'candidate_summary': candidate_summary,
            'constraint_audit': {
                'n_total': constraint_audit.get('n_total', 0),
                'n_feasible': constraint_audit.get('n_feasible', 0),
                'feasible_ratio': constraint_audit.get('feasible_ratio', 0.0),
                'per_constraint_stats': constraint_audit.get('per_constraint_stats', {})
            },
            'explanation': {
                'selected_id': explanation.get('selected_id'),
                'selected_cost': explanation.get('selected_cost'),
                'is_feasible': explanation.get('is_feasible'),
                'selection_reason': explanation.get('selection_reason'),
                'top_2_margin': explanation.get('top_2_margin', None)
            }
        }
        
        # Add minimal relaxation if present
        if 'minimal_relaxation' in constraint_audit:
            log_entry['constraint_audit']['minimal_relaxation'] = constraint_audit['minimal_relaxation']
        
        self._write_jsonl(self.ticks_file, log_entry)
        self.tick_count += 1
    
    def log_event(
        self,
        event_type: str,
        timestep: int,
        state: np.ndarray,
        selected_action: np.ndarray,
        candidate_summary: Dict,
        constraint_audit: Dict,
        explanation: Dict,
        additional_info: Optional[Dict] = None
    ):
        """
        Log critical events only
        
        Event types:
            - 'no_feasible_actions': N_feasible == 0
            - 'near_collision': Very close to obstacle
            - 'constraint_violation': Selected action violates constraints
            - 'narrow_margin': Very small margin between top candidates
        
        Args:
            event_type: Type of event
            timestep: Current simulation step
            state: Current state vector
            selected_action: Selected action
            candidate_summary: Candidate generation summary
            constraint_audit: Constraint audit results
            explanation: Decision explanation
            additional_info: Additional event-specific information
        """
        log_entry = {
            'event_type': event_type,
            'event_id': self.event_count,
            'timestep': timestep,
            'timestamp': datetime.now().isoformat(),
            'state': {
                'position': state[:3].tolist(),
                'velocity': state[3:6].tolist(),
            },
            'selected_action': selected_action.tolist() if isinstance(selected_action, np.ndarray) else selected_action,
            'candidate_summary': candidate_summary,
            'constraint_audit': {
                'n_total': constraint_audit.get('n_total', 0),
                'n_feasible': constraint_audit.get('n_feasible', 0),
                'feasible_ratio': constraint_audit.get('feasible_ratio', 0.0),
                'per_constraint_stats': constraint_audit.get('per_constraint_stats', {}),
            },
            'explanation': explanation
        }
        
        # Add minimal relaxation if present
        if 'minimal_relaxation' in constraint_audit:
            log_entry['constraint_audit']['minimal_relaxation'] = constraint_audit['minimal_relaxation']
        
        if additional_info:
            log_entry['additional_info'] = additional_info
        
        self._write_jsonl(self.events_file, log_entry)
        self.event_count += 1
    
    def _write_jsonl(self, filepath: str, entry: Dict):
        """Write a single JSON line to file"""
        with open(filepath, 'a', encoding='utf-8') as f:
            json.dump(entry, f, ensure_ascii=False)
            f.write('\n')
    
    def close(self):
        """Close log files (for completeness, JSONL doesn't need explicit close)"""
        pass
    
    def get_stats(self) -> Dict:
        """Get logging statistics"""
        return {
            'ticks_logged': self.tick_count,
            'events_logged': self.event_count,
            'ticks_file': self.ticks_file,
            'events_file': self.events_file
        }


def convert_numpy_types(obj):
    """
    Recursively convert numpy types to native Python types for JSON serialization
    """
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    elif isinstance(obj, dict):
        return {key: convert_numpy_types(value) for key, value in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy_types(item) for item in obj]
    else:
        return obj
