"""
Natural language generation for forensic triage summaries.

Combines DWA decision rationale and constraint-based feasibility analysis
to generate human-readable summaries for post-hoc investigation triage.
Purpose: Aid investigators in quickly determining inevitability vs choice error,
NOT to determine root causes.
"""

from typing import Dict, Optional


def _format_float(value: Optional[float], digits: int = 3) -> str:
    if value is None:
        return "N/A"
    try:
        return f"{float(value):.{digits}f}"
    except (TypeError, ValueError):
        return "N/A"


def build_natural_language_summary(
    report: Dict,
    last_event: Optional[Dict],
    last_tick: Optional[Dict],
    language: str = "ko",
) -> str:
    """
    Build a short natural language summary that combines:
    - DWA decision rationale
    - Constraint-based feasibility/inevitability evidence
    """
    if language != "ko":
        language = "ko"

    scenario = report.get("scenario")
    gt_label = report.get("gt_label")
    predicted = report.get("predicted_label", gt_label)
    expected_xai = report.get("expected_xai")

    # Decision-level info
    decision_margin = None
    selected_cost = None
    score_breakdown = None
    
    # Try to get from last_event first (trigger point), then fallback to last_tick
    if last_event:
        ring_buffer = last_event.get("ring_buffer", [])
        if ring_buffer:
            last_tick_in_event = ring_buffer[-1]
            decision_margin = last_tick_in_event.get("metrics", {}).get("decision_margin")
            selected_cost = (
                last_tick_in_event.get("dwa", {}).get("selected_cost")
                or last_tick_in_event.get("dwa", {}).get("selected_cost_total")
            )
            score_breakdown = (
                last_tick_in_event.get("dwa", {}).get("score_breakdown")
                or last_tick_in_event.get("dwa", {}).get("cost_terms", {})
            )
    
    # Fallback to last_tick if not found in last_event
    if not score_breakdown and last_tick:
        decision_margin = last_tick.get("metrics", {}).get("decision_margin")
        selected_cost = (
            last_tick.get("dwa", {}).get("selected_cost")
            or last_tick.get("dwa", {}).get("selected_cost_total")
        )
        score_breakdown = (
            last_tick.get("dwa", {}).get("score_breakdown")
            or last_tick.get("dwa", {}).get("cost_terms", {})
        )

    # Constraint-level info
    n_feasible = None
    dist_min = None
    ttc = None
    minimal_relaxation = None
    dominant_constraint = None
    if last_event:
        n_feasible = last_event.get("metrics", {}).get("n_feasible")
        dist_min = last_event.get("metrics", {}).get("dist_min")
        ttc = last_event.get("metrics", {}).get("ttc")
        minimal_relaxation = last_event.get("constraints", {}).get("minimal_relaxation")
        per_stats = last_event.get("constraints", {}).get("per_constraint_stats", {})
        if per_stats:
            dominant_constraint = max(
                per_stats.items(),
                key=lambda x: x[1].get("violation_ratio", 0.0)
            )[0]

    # Top-K alternative info
    top_k = None
    selected_vs_safer = None
    if last_event:
        top_k = last_event.get("top_k", [])
        selected_vs_safer = last_event.get("selected_vs_safer_alternative", {})

    lines = []
    lines.append(f"[시나리오 {scenario}] GT={gt_label}, 예측={predicted}")
    if expected_xai:
        lines.append(f"기대 XAI: {expected_xai}")

    # DWA rationale with cost term breakdown
    if score_breakdown:
        goal_term = score_breakdown.get("goal_distance")
        obs_term = score_breakdown.get("obstacle_penalty")
        vel_term = score_breakdown.get("velocity_magnitude")
        yaw_term = score_breakdown.get("yaw_rate_magnitude")
        
        lines.append(
            "DWA 선택 근거: "
            f"cost={_format_float(selected_cost)}, "
            f"margin={_format_float(decision_margin)}, "
            f"goal={_format_float(goal_term)}, "
            f"obs_pen={_format_float(obs_term)}, "
            f"vel={_format_float(vel_term)}, "
            f"yaw={_format_float(yaw_term)}"
        )
        
        # RQ3: Cost term dominance analysis
        # Typical weights from S2: goal=35.0, obstacle=0.05, velocity=0.02, yaw_rate=0.02
        if goal_term is not None and obs_term is not None:
            goal_weight = 35.0  # Default S2 weight
            obs_weight = 0.05
            goal_weighted = goal_term * goal_weight
            obs_weighted = obs_term * obs_weight
            
            # Show dominance if goal term is significantly larger
            if goal_weighted > obs_weighted * 10 or (obs_weighted == 0 and goal_weighted > 0):
                lines.append(
                    f"비용 항 우세도: goal_term({_format_float(goal_weighted)}) >> "
                    f"obstacle_term({_format_float(obs_weighted)}) "
                    f"→ 목표 지향성 비용 항 우세 (goal term dominance)"
                )

    # Constraint-based feasibility
    if last_event:
        lines.append(
            "제약 기반 판단: "
            f"n_feasible={n_feasible}, "
            f"dist_min={_format_float(dist_min)}, "
            f"TTC={_format_float(ttc)}, "
            f"dominant={dominant_constraint}"
        )
        if minimal_relaxation:
            relax_pairs = ", ".join(
                f"{k}={_format_float(v)}" for k, v in minimal_relaxation.items()
            )
            lines.append(f"최소 완화 Δ: {relax_pairs}")

    # Alternatives - RQ3: Selected vs Safer Alternative comparison
    if selected_vs_safer and selected_vs_safer.get("safer_feasible_alternative_exists"):
        safer_margin = selected_vs_safer.get("decision_margin")
        safer_cost = selected_vs_safer.get("safer_cost_total")
        safer_rank = selected_vs_safer.get("safer_alternative_rank")
        
        lines.append(
            f"RQ3 결정 마진: 선택된 행동(cost={_format_float(selected_cost)}) vs "
            f"더 안전한 feasible 대안(cost={_format_float(safer_cost)}) = "
            f"{_format_float(safer_margin)} (Top-K 내 순위: {safer_rank})"
        )
        lines.append(
            f"→ 위험한 행동이 더 낮은 비용({_format_float(selected_cost)})으로 선택되어 "
            f"안전한 대안({_format_float(safer_cost)})보다 {_format_float(safer_margin)}만큼 우선됨"
        )
    elif top_k:
        best_alt = None
        for cand in top_k:
            if cand.get("is_feasible"):
                best_alt = cand
                break
        if best_alt:
            best_cost = best_alt.get("cost") or best_alt.get("cost_total")
            lines.append(
                "대안: feasible 후보 존재 "
                f"(id={best_alt.get('candidate_id')}, cost={_format_float(best_cost)})."
            )
        else:
            lines.append("대안: Top-K 모두 infeasible.")

    return "\n".join(lines)
