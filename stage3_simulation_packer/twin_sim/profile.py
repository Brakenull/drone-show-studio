"""drone_profile.json loader with the 3-tier fallback of docs/3-phase-3.md §1.3.

    Tier 1: --profile <path>                (project-specific, optional)
    Tier 2: config/drone_profile.json       (Phase 3 default)
    Tier 3: packer/include/drone_constants.hpp constexprs

Tiers are merged key-by-key (a tier-1 file that only overrides `mass_kg`
keeps every other value from tier 2, and anything missing from tier 2 comes
from tier 3). Tier 3 is parsed straight out of the C++ header via its
`// profile: <key.path>` tags, so the packer and the simulator can never
disagree about a compiled-in default.
"""

from __future__ import annotations

import copy
import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

STAGE3_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PROFILE_PATH = STAGE3_ROOT / "config" / "drone_profile.json"
CONSTANTS_HEADER_PATH = STAGE3_ROOT / "packer" / "include" / "drone_constants.hpp"

TIER_CLI = "cli"
TIER_DEFAULT_FILE = "default_file"
TIER_CONSTANTS = "constants"

_TAGGED_LINE = re.compile(r"=\s*(?P<value>.+?)\s*;\s*//\s*profile:\s*(?P<key>[\w.]+)")
_BRACED_LINE = re.compile(r"\{(?P<value>[^}]*)\}\s*;\s*//\s*profile:\s*(?P<key>[\w.]+)")


def _parse_scalar(text: str) -> Any:
    text = text.strip()
    if text.startswith('"') and text.endswith('"'):
        return text[1:-1]
    number = float(text)
    return int(number) if number.is_integer() and "." not in text else number


def parse_constants_header(path: Path = CONSTANTS_HEADER_PATH) -> dict[str, Any]:
    """Build the tier-3 nested profile dict from drone_constants.hpp."""
    profile: dict[str, Any] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        braced = _BRACED_LINE.search(line)
        if braced:
            key = braced.group("key")
            value: Any = [float(v) for v in braced.group("value").split(",")]
        else:
            tagged = _TAGGED_LINE.search(line)
            if not tagged:
                continue
            key = tagged.group("key")
            value = _parse_scalar(tagged.group("value"))
        _set_path(profile, key, value)
    if not profile:
        raise ValueError(f"No `// profile:` tagged constants found in {path}")
    return profile


def _set_path(tree: dict[str, Any], dotted: str, value: Any) -> None:
    parts = dotted.split(".")
    node = tree
    for part in parts[:-1]:
        node = node.setdefault(part, {})
    node[parts[-1]] = value


def _flatten(tree: dict[str, Any], prefix: str = "") -> dict[str, Any]:
    flat: dict[str, Any] = {}
    for key, value in tree.items():
        dotted = f"{prefix}{key}"
        if isinstance(value, dict):
            flat.update(_flatten(value, dotted + "."))
        else:
            flat[dotted] = value
    return flat


@dataclass
class DroneProfile:
    values: dict[str, Any]
    # dotted key -> which tier supplied it (cli / default_file / constants)
    sources: dict[str, str] = field(default_factory=dict)

    def get(self, dotted: str) -> Any:
        node: Any = self.values
        for part in dotted.split("."):
            node = node[part]
        return node

    @property
    def name(self) -> str:
        return str(self.values["profile_name"])

    def to_json(self) -> dict[str, Any]:
        return copy.deepcopy(self.values)


def _read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        data = json.load(fh)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: drone profile must be a JSON object")
    return data


def _merge_tier(base_flat: dict[str, Any], sources: dict[str, str], tier: dict[str, Any], tier_name: str,
                origin: Path) -> None:
    for dotted, value in _flatten(tier).items():
        if dotted not in base_flat:
            # A typo'd key in a safety-relevant config must not silently fall
            # back to a default.
            raise ValueError(f"{origin}: unknown drone profile key '{dotted}'")
        expected = base_flat[dotted]
        if isinstance(expected, list):
            if not isinstance(value, list) or len(value) != len(expected):
                raise ValueError(f"{origin}: '{dotted}' must be a list of {len(expected)} numbers")
            value = [float(v) for v in value]
        elif isinstance(expected, str):
            value = str(value)
        elif isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{origin}: '{dotted}' must be a number, got {value!r}")
        base_flat[dotted] = value
        sources[dotted] = tier_name


def _validate(flat: dict[str, Any], origin: str) -> None:
    for dotted, value in flat.items():
        if dotted == "profile_name" or dotted.startswith("tolerances."):
            continue
        values = value if isinstance(value, list) else [value]
        if any(v <= 0 for v in values):
            raise ValueError(f"{origin}: '{dotted}' must be > 0, got {value!r}")
    if flat["battery.cutoff_voltage_v"] >= flat["battery.nominal_voltage_v"]:
        raise ValueError(f"{origin}: battery.cutoff_voltage_v must be below nominal_voltage_v")
    if not 0.0 < flat["battery.initial_soc"] <= 1.0:
        raise ValueError(f"{origin}: battery.initial_soc must be in (0, 1]")
    if not 0.0 < flat["battery.powertrain_efficiency"] <= 1.0:
        raise ValueError(f"{origin}: battery.powertrain_efficiency must be in (0, 1]")


def load_profile(profile_path: str | Path | None = None, *,
                 default_path: Path = DEFAULT_PROFILE_PATH,
                 constants_header: Path = CONSTANTS_HEADER_PATH) -> DroneProfile:
    flat = _flatten(parse_constants_header(constants_header))
    sources = {key: TIER_CONSTANTS for key in flat}

    if default_path.is_file():
        _merge_tier(flat, sources, _read_json(default_path), TIER_DEFAULT_FILE, default_path)

    if profile_path is not None:
        path = Path(profile_path)
        if not path.is_file():
            # An explicitly requested profile that doesn't exist is an
            # operator error, not a reason to fall back silently.
            raise FileNotFoundError(f"--profile file not found: {path}")
        _merge_tier(flat, sources, _read_json(path), TIER_CLI, path)

    _validate(flat, str(profile_path or default_path))
    values: dict[str, Any] = {}
    for dotted, value in flat.items():
        _set_path(values, dotted, value)
    return DroneProfile(values=values, sources=sources)
