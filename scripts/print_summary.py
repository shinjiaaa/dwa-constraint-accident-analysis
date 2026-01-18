"""
Print human-friendly XAI summaries from outputs.
"""

import json
import os
from src.env import load_dotenv
from src.llm import build_llm_prompt, call_openai_llm
import argparse
from pathlib import Path


def load_last_event(events_path: Path):
    if not events_path.exists():
        return None
    lines = events_path.read_text(encoding="utf-8").strip().splitlines()
    if not lines:
        return None
    return json.loads(lines[-1])


def print_summary(report_path: Path, events_path: Path, llm: bool, model: str):
    if report_path.exists():
        report = json.loads(report_path.read_text(encoding="utf-8"))
        print("\n==== XAI 요약 (최종) ====")
        print(report.get("summary_text", "요약이 없습니다."))
    else:
        print("\n[경고] report.json이 없습니다.")

    last_event = load_last_event(events_path)
    if last_event and "summary_text" in last_event:
        print("\n==== XAI 요약 (이벤트) ====")
        print(last_event["summary_text"])
    else:
        print("\n[경고] events.jsonl 요약이 없습니다.")

    if llm and report_path.exists():
        report = json.loads(report_path.read_text(encoding="utf-8"))
        prompt = build_llm_prompt(report, last_event, None)
        llm_text = call_openai_llm(prompt, model=model)
        print("\n==== LLM 요약 ====")
        print(llm_text)


def main():
    load_dotenv()
    parser = argparse.ArgumentParser(description="Print human-friendly XAI summaries.")
    parser.add_argument("--scenario", required=True, choices=["s1", "s3"])
    parser.add_argument("--out", default="outputs")
    parser.add_argument("--llm", action="store_true", default=False)
    parser.add_argument("--llm-model", default="gpt-4o-mini")
    args = parser.parse_args()

    base = Path(args.out) / args.scenario
    report_path = base / "report.json"
    events_path = base / "events.jsonl"
    print_summary(report_path, events_path, llm=args.llm, model=args.llm_model)


if __name__ == "__main__":
    main()
