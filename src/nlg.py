"""
Natural language generation for combined DWA + constraint-based XAI.
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
    if last_tick:
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
    if last_event:
        top_k = last_event.get("top_k", [])

    lines = []
    lines.append(f"[시나리오 {scenario}] GT={gt_label}, 예측={predicted}")
    if expected_xai:
        lines.append(f"기대 XAI: {expected_xai}")

    # DWA rationale
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

    # Alternatives
    if top_k:
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
