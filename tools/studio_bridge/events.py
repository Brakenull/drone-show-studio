"""NDJSON event stream. Stdout carries events only."""

from __future__ import annotations

import json
import math
import os
import sys
from typing import IO, Any

EXIT_OK = 0
EXIT_CRITERION = 1  # safety / Monte Carlo criterion failed
EXIT_INPUT = 2
EXIT_INTERNAL = 3

_events: IO[str] = sys.stdout


def reserve_stdout() -> None:
    """Keep stdout (fd 1) for events only. Afterwards print() and anything a native library writes to
    fd 1 (Warp prints its start-up banner there) go to stderr, which the shell logs as text."""
    global _events
    sys.stdout.flush()
    _events = os.fdopen(os.dup(1), "w", encoding="utf-8", newline="\n")
    os.dup2(2, 1)
    sys.stdout = sys.stderr


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
    _events.write(json.dumps(_clean({"type": event_type, **fields}), separators=(",", ":")) + "\n")
    _events.flush()


def log(message: str) -> None:
    """Free-form diagnostics go to stderr; the shell stores them in log.ndjson."""
    sys.stderr.write(message + "\n")
    sys.stderr.flush()
