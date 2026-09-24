"""Repo locations and the drone_core import (same search order as export_stage2_trajectories.py)."""

from __future__ import annotations

import os
import sys
from pathlib import Path
from types import ModuleType

REPO_ROOT = Path(__file__).resolve().parents[2]
SCHEMA_PATH = REPO_ROOT / "schemas" / "project_intermediate.schema.json"
PACKER_CANDIDATES = [
    REPO_ROOT / "stage3_simulation_packer" / "build" / "pack_to_binary.exe",
    REPO_ROOT / "stage3_simulation_packer" / "build" / "pack_to_binary",
]

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def extension_dirs() -> list[Path]:
    explicit = os.environ.get("STUDIO_STAGE2_EXTENSION_DIR")
    dirs = [Path(explicit)] if explicit else []
    return dirs + [REPO_ROOT / "stage2_core_engine" / "build"]


def find_extension_dir() -> Path | None:
    for candidate in extension_dirs():
        if list(candidate.glob("drone_core*.pyd")) or list(candidate.glob("drone_core*.so")):
            return candidate
    return None


def import_drone_core() -> ModuleType:
    """Raises ImportError with a readable reason when the extension is missing or built for another Python."""
    ext_dir = find_extension_dir()
    if ext_dir is None:
        raise ImportError("drone_core extension not found; build stage2_core_engine first")
    if str(ext_dir) not in sys.path:
        sys.path.insert(0, str(ext_dir))
    import drone_core

    return drone_core


def find_packer() -> Path | None:
    return next((p for p in PACKER_CANDIDATES if p.exists()), None)
