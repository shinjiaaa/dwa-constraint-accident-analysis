"""
Operational logging utilities for ticks/events JSONL.
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
