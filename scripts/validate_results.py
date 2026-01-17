"""
Validation script for accident analysis results
Automatically validates log files against expected outcomes
"""

import json
import os
import sys
import argparse
from pathlib import Path
from typing import Dict, List


def load_jsonl(filepath: str) -> List[Dict]:
    """Load JSONL file"""
    data = []
    with open(filepath, 'r', encoding='utf-8') as f:
        for line in f:
            if line.strip():
                data.append(json.loads(line))
    return data


def validate_ticks_log(ticks_file: str) -> Dict[str, bool]:
    """Validate ticks.jsonl file"""
    results = {
        'file_exists': False,
        'parsable': False,
        'has_required_fields': False,
        'timestep_continuity': False,
        'feasible_ratio_valid': False
    }
    
    if not os.path.exists(ticks_file):
        return results
    
    results['file_exists'] = True
    
    try:
        data = load_jsonl(ticks_file)
        results['parsable'] = True
        
        if not data:
            return results
        
        # Check required fields
        required_fields = [
            'timestep', 'state', 'selected_action',
            'constraint_audit', 'explanation'
        ]
        sample = data[0]
        results['has_required_fields'] = all(
            field in sample for field in required_fields
        )
        
        # Check timestep continuity
        timesteps = [entry['timestep'] for entry in data]
        expected_timesteps = list(range(len(timesteps)))
        results['timestep_continuity'] = timesteps == expected_timesteps
        
        # Check feasible_ratio is valid (0.0 to 1.0)
        feasible_ratios = [
            entry['constraint_audit'].get('feasible_ratio', -1)
            for entry in data
        ]
        results['feasible_ratio_valid'] = all(
            0.0 <= r <= 1.0 for r in feasible_ratios
        )
        
    except Exception as e:
        print(f"Error validating ticks: {e}")
    
    return results


def validate_events_log(events_file: str, scenario: str) -> Dict[str, bool]:
    """Validate events.jsonl file"""
    results = {
        'file_exists': False,
        'parsable': False,
        'has_event_types': False,
        'has_no_feasible_events': False,
        'has_minimal_relaxation': False
    }
    
    if not os.path.exists(events_file):
        return results
    
    results['file_exists'] = True
    
    try:
        data = load_jsonl(events_file)
        results['parsable'] = True
        
        if not data:
            return results
        
        # Check event types exist
        event_types = [entry.get('event_type') for entry in data]
        results['has_event_types'] = len(event_types) > 0
        
        # Scenario-specific checks
        if scenario == 'A':
            # Scenario A should have no_feasible_actions events
            no_feasible = [
                e for e in data
                if e.get('event_type') == 'no_feasible_actions'
            ]
            results['has_no_feasible_events'] = len(no_feasible) > 0
            
            # Should have minimal_relaxation
            has_relaxation = any(
                'minimal_relaxation' in e.get('constraint_audit', {})
                for e in no_feasible
            )
            results['has_minimal_relaxation'] = has_relaxation
        
        elif scenario == 'B':
            # Scenario B should have constraint_violation events
            violations = [
                e for e in data
                if e.get('event_type') == 'constraint_violation'
            ]
            results['has_constraint_violations'] = len(violations) > 0
        
        elif scenario == 'C':
            # Scenario C should have near_collision events
            collisions = [
                e for e in data
                if e.get('event_type') == 'near_collision'
            ]
            results['has_near_collisions'] = len(collisions) > 0
        
    except Exception as e:
        print(f"Error validating events: {e}")
    
    return results


def validate_scenario_results(log_dir: str, scenario: str) -> Dict:
    """Validate results for a specific scenario"""
    log_path = Path(log_dir)
    
    # Find most recent log files
    ticks_files = list(log_path.glob(f"ticks_*.jsonl"))
    events_files = list(log_path.glob(f"events_*.jsonl"))
    
    if not ticks_files:
        return {
            'error': 'No ticks.jsonl files found',
            'valid': False
        }
    
    # Use most recent
    ticks_file = max(ticks_files, key=os.path.getmtime)
    events_file = max(events_files, key=os.path.getmtime) if events_files else None
    
    # Validate
    ticks_results = validate_ticks_log(str(ticks_file))
    events_results = validate_events_log(str(events_file), scenario) if events_file else {}
    
    # Overall validation
    all_checks = {
        **ticks_results,
        **events_results
    }
    
    valid = all(
        v for k, v in all_checks.items()
        if k not in ['file_exists', 'parsable']  # These are prerequisites
    )
    
    return {
        'valid': valid,
        'ticks_file': str(ticks_file),
        'events_file': str(events_file) if events_file else None,
        'ticks_validation': ticks_results,
        'events_validation': events_results,
        'all_checks': all_checks
    }


def print_validation_report(results: Dict):
    """Print validation report"""
    print("=" * 80)
    print("Validation Report")
    print("=" * 80)
    
    if 'error' in results:
        print(f"❌ Error: {results['error']}")
        return
    
    print(f"\nTicks File: {results['ticks_file']}")
    print(f"Events File: {results['events_file']}")
    
    print("\nTicks Validation:")
    for check, passed in results['ticks_validation'].items():
        status = "✓" if passed else "✗"
        print(f"  {status} {check}")
    
    if results['events_validation']:
        print("\nEvents Validation:")
        for check, passed in results['events_validation'].items():
            status = "✓" if passed else "✗"
            print(f"  {status} {check}")
    
    print("\n" + "=" * 80)
    if results['valid']:
        print("✓ Validation PASSED")
    else:
        print("✗ Validation FAILED")
        print("\nFailed checks:")
        for check, passed in results['all_checks'].items():
            if not passed:
                print(f"  - {check}")
    print("=" * 80)


def main():
    parser = argparse.ArgumentParser(description='Validate accident analysis results')
    parser.add_argument(
        '--scenario',
        type=str,
        required=True,
        choices=['A', 'B', 'C'],
        help='Scenario to validate'
    )
    parser.add_argument(
        '--log-dir',
        type=str,
        default='logs',
        help='Log directory'
    )
    
    args = parser.parse_args()
    
    results = validate_scenario_results(args.log_dir, args.scenario)
    print_validation_report(results)
    
    return 0 if results.get('valid', False) else 1


if __name__ == "__main__":
    exit(main())
