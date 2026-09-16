"""Zip `stage1_designer/` into an installable Blender add-on package.

Usage:
    python tools/scripts/build_blender_addon.py [output_dir]

Produces `<output_dir>/stage1_designer.zip` (default output_dir: `dist/`).
"""

from __future__ import annotations

import sys
import zipfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ADDON_DIR = REPO_ROOT / "stage1_designer"
EXCLUDE_SUFFIXES = {".pyc"}
EXCLUDE_DIR_NAMES = {"__pycache__"}


def build(output_dir: Path) -> Path:
    if not ADDON_DIR.is_dir():
        raise SystemExit(f"Add-on source not found: {ADDON_DIR}")

    output_dir.mkdir(parents=True, exist_ok=True)
    zip_path = output_dir / "stage1_designer.zip"

    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for path in sorted(ADDON_DIR.rglob("*")):
            if path.is_dir():
                continue
            if path.suffix in EXCLUDE_SUFFIXES:
                continue
            if any(part in EXCLUDE_DIR_NAMES for part in path.parts):
                continue
            arcname = Path(ADDON_DIR.name) / path.relative_to(ADDON_DIR)
            zf.write(path, arcname)

    return zip_path


if __name__ == "__main__":
    out_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else REPO_ROOT / "dist"
    result = build(out_dir)
    print(f"Built {result}")
