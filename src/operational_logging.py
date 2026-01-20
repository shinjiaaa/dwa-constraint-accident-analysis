"""
Operational logging utilities for ticks/events JSONL.

Implements the Replay Contract (재현 최소 로그 계약):
- ticks.jsonl: Continuous operational data (low-bit, every timestep)
  Captures state, action, DWA costs, constraint stats for replay
- events.jsonl: Event-triggered snapshots (ring buffer, Top-K, minimal relaxation)
  Captures decision rationale and feasibility analysis at critical moments

This logging schema enables post-hoc replay of autonomous decision-making
to answer two key questions:
1. Inevitability (불가피성): Did safe alternatives exist?
2. Choice error (선택오류): Why was a risky action chosen despite safer alternatives?
"""

import json
import os
from typing import Dict


class OperationalLogger:
    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        self.ticks_path = os.path.join(self.output_dir, "ticks.jsonl")
        self.events_path = os.path.join(self.output_dir, "events.jsonl")

    def log_tick(self, entry: Dict):
        self._write(self.ticks_path, entry)

    def log_event(self, entry: Dict):
        self._write(self.events_path, entry)

    def _write(self, path: str, entry: Dict):
        with open(path, "a", encoding="utf-8") as f:
            json.dump(entry, f, ensure_ascii=False)
            f.write("\n")
