"""NDJSON event stream (docs/5-studio_gui.md §4). Stdout carries events only."""

from __future__ import annotations

import json
import math
import sys
from typing import Any

EXIT_OK = 0
EXIT_CRITERION = 1  # safety / Monte Carlo criterion failed
EXIT_INPUT = 2
EXIT_INTERNAL = 3


def _clean(value: Any) -> Any:
    # JSON has no inf/nan; the UI treats null as "not measured".
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    return value


def emit(event_type: str, **fields: Any) -> None:
    sys.stdout.write(json.dumps(_clean({"type": event_type, **fields}), separators=(",", ":")) + "\n")
    sys.stdout.flush()


def log(message: str) -> None:
    """Free-form diagnostics go to stderr; the shell stores them in log.ndjson."""
    sys.stderr.write(message + "\n")
    sys.stderr.flush()
