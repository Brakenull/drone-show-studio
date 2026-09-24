"""Run folders (docs/5-studio_gui.md §3): one folder per run, `run.json` is the record."""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Any

from .paths import REPO_ROOT

RUN_FILE = "run.json"
STATUSES = ("not_run", "running", "succeeded", "failed_safety", "failed_input", "failed_error", "cancelled")


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def _git_commit() -> str | None:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO_ROOT, capture_output=True,
                             text=True, timeout=5)
        return out.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def write_json_atomic(path: Path, data: Any, indent: int | None = 2) -> None:
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=indent)
    os.replace(tmp, path)


def read_run(run_dir: Path) -> dict[str, Any]:
    with (run_dir / RUN_FILE).open("r", encoding="utf-8") as fh:
        return json.load(fh)


def update_run(run_dir: Path, section: str, **fields: Any) -> dict[str, Any]:
    record = read_run(run_dir)
    record.setdefault(section, {}).update(fields)
    write_json_atomic(run_dir / RUN_FILE, record)
    return record


def update_stage3(run_dir: Path, part: str, **fields: Any) -> dict[str, Any]:
    """Like update_run for run.json's nested stage3.monte_carlo / stage3.pack records."""
    record = read_run(run_dir)
    record.setdefault("stage3", {}).setdefault(part, {}).update(fields)
    write_json_atomic(run_dir / RUN_FILE, record)
    return record


def create_run(source: Path, runs_dir: Path, phase1: dict[str, Any], copy_from: Path | None = None,
               overrides: dict[str, Any] | None = None) -> Path:
    """Copy the (already validated) Phase 1 file into a fresh run folder. `copy_from` is an existing run
    being copied (to try other planner settings on the same show): its saved settings come along unless
    `overrides` gives the new run's planner settings directly."""
    raw = source.read_bytes()
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    # A copy keeps the original run's name and source path, not those of the copied input file.
    original = read_run(copy_from) if copy_from else None
    name = copy_from.name.split("_", 1)[-1] if copy_from else source.stem
    stem = re.sub(r"[^A-Za-z0-9_.-]+", "_", name)[:60] or "show"
    runs_dir.mkdir(parents=True, exist_ok=True)
    run_dir = runs_dir / f"{stamp}_{stem}"
    suffix = 1
    while run_dir.exists():
        suffix += 1
        run_dir = runs_dir / f"{stamp}_{stem}-{suffix}"
    (run_dir / "input").mkdir(parents=True)
    (run_dir / "input" / "phase1.json").write_bytes(raw)

    meta = phase1.get("project_metadata", {})
    record = {
        "run_id": run_dir.name,
        "created_at": now_iso(),
        "input": {
            "source_path": original["input"]["source_path"] if original else str(source),
            "sha256": hashlib.sha256(raw).hexdigest(),
            "fleet_size": meta.get("fleet_size"),
            "keyframes": [kf.get("shape_name") for kf in phase1.get("keyframes", [])],
        },
        "stage2": {"status": "not_run"},
        **({"copied_from": copy_from.name} if copy_from else {}),
        "stage3": {"monte_carlo": {"status": "not_run"}, "pack": {"status": "not_run"}},
        "versions": {"git_commit": _git_commit()},
    }
    write_json_atomic(run_dir / RUN_FILE, record)
    settings = copy_from / "stage2" / "config_overrides.json" if copy_from else None
    if overrides is not None:
        (run_dir / "stage2").mkdir()
        write_json_atomic(run_dir / "stage2" / "config_overrides.json", overrides)
    elif settings and settings.exists():
        (run_dir / "stage2").mkdir()
        (run_dir / "stage2" / "config_overrides.json").write_bytes(settings.read_bytes())
    return run_dir
