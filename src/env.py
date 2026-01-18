"""
Simple .env loader (no external deps).
"""

import os
from typing import Optional


def load_dotenv(path: Optional[str] = ".env") -> None:
    """
    Load key=value pairs from a .env file into os.environ.
    Ignores comments and empty lines.
    """
    if not path or not os.path.exists(path):
        return
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            key = key.strip()
            value = value.strip().strip("\"'")  # strip quotes
            if key and key not in os.environ:
                os.environ[key] = value
