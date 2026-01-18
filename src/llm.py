"""
LLM integration for natural language XAI summaries (OpenAI).
"""

import json
import os
from typing import Dict, Optional
from urllib.request import Request, urlopen
from urllib.error import HTTPError, URLError


def build_llm_prompt(
    report: Dict,
    last_event: Optional[Dict],
    last_tick: Optional[Dict],
) -> str:
    """
    Build a concise prompt for LLM summarization.
    """
    scenario = report.get("scenario")
    gt_label = report.get("gt_label")
    expected = report.get("expected_xai")

    event_metrics = last_event.get("metrics", {}) if last_event else {}
    event_constraints = last_event.get("constraints", {}) if last_event else {}
    top_k = last_event.get("top_k", []) if last_event else []

    tick_metrics = last_tick.get("metrics", {}) if last_tick else {}
    tick_dwa = last_tick.get("dwa", {}) if last_tick else {}
    tick_constraints = last_tick.get("constraints", {}) if last_tick else {}

    payload = {
        "scenario": scenario,
        "gt_label": gt_label,
        "expected_xai": expected,
        "event_metrics": event_metrics,
        "event_constraints": event_constraints,
        "top_k": top_k[:3],
        "tick_metrics": tick_metrics,
        "tick_dwa": tick_dwa,
        "tick_constraints": tick_constraints,
    }

    return (
        "너는 로보틱스 사고 분석 전문가다. 아래 데이터를 근거로 "
        "한국어 자연어 요약을 작성해라. "
        "요약은 5~8문장으로, "
        "1) 사고 원인(필연/오판 여부), "
        "2) DWA 결정 이유(비용 항/마진), "
        "3) 제약 기반 판단(가능/불가능, 최소 완화), "
        "4) 대안 후보 유무를 포함해라.\n\n"
        f"DATA(JSON): {json.dumps(payload, ensure_ascii=False)}"
    )


def call_openai_llm(prompt: str, model: str = "gpt-4o-mini") -> str:
    """
    Call OpenAI Responses API with the given prompt.
    """
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        return "[LLM] OPENAI_API_KEY가 설정되지 않았습니다."

    url = "https://api.openai.com/v1/responses"
    body = {
        "model": model,
        "input": [
            {
                "role": "user",
                "content": [{"type": "text", "text": prompt}],
            }
        ],
        "temperature": 0.2,
    }
    data = json.dumps(body).encode("utf-8")
    req = Request(url, data=data, headers={
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}",
    })

    try:
        with urlopen(req, timeout=30) as resp:
            resp_data = json.loads(resp.read().decode("utf-8"))
    except HTTPError as e:
        return f"[LLM] HTTPError: {e.code}"
    except URLError as e:
        return f"[LLM] URLError: {e.reason}"
    except Exception as e:
        return f"[LLM] Error: {e}"

    # Try to extract output text
    if "output_text" in resp_data:
        return resp_data["output_text"]
    # Fallback for nested format
    try:
        return resp_data["output"][0]["content"][0]["text"]
    except Exception:
        return "[LLM] 응답 파싱 실패"
