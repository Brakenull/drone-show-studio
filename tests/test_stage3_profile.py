import json

import pytest

from stage3_simulation_packer.twin_sim.profile import (
    DEFAULT_PROFILE_PATH,
    TIER_CLI,
    TIER_CONSTANTS,
    TIER_DEFAULT_FILE,
    _flatten,
    load_profile,
    parse_constants_header,
)


def test_constants_header_and_default_json_define_identical_values():
    # Tier 3 (C++ constexpr) and tier 2 (JSON) must never drift apart.
    tier3 = _flatten(parse_constants_header())
    tier2 = _flatten(json.loads(DEFAULT_PROFILE_PATH.read_text(encoding="utf-8")))
    assert tier3.keys() == tier2.keys()
    for key, value in tier2.items():
        assert tier3[key] == pytest.approx(value), key


def test_default_load_uses_default_file():
    profile = load_profile()
    assert profile.name == "Standard_Show_Quad_v1"
    assert profile.get("physical.mass_kg") == pytest.approx(0.55)
    assert profile.get("battery.cells_s") == 4
    assert set(profile.sources.values()) == {TIER_DEFAULT_FILE}


def test_cli_profile_overrides_only_its_keys(tmp_path):
    override = tmp_path / "heavy.json"
    override.write_text(json.dumps({"profile_name": "Heavy", "physical": {"mass_kg": 0.8}}))
    profile = load_profile(override)
    assert profile.name == "Heavy"
    assert profile.get("physical.mass_kg") == pytest.approx(0.8)
    assert profile.sources["physical.mass_kg"] == TIER_CLI
    assert profile.get("physical.arm_length_m") == pytest.approx(0.16)
    assert profile.sources["physical.arm_length_m"] == TIER_DEFAULT_FILE


def test_missing_default_file_falls_back_to_constants(tmp_path):
    profile = load_profile(default_path=tmp_path / "absent.json")
    assert set(profile.sources.values()) == {TIER_CONSTANTS}
    assert profile.get("controller_gains.pos_p") == [2.5, 2.5, 3.0]


def test_partial_default_file_fills_missing_keys_from_constants(tmp_path):
    partial = tmp_path / "partial.json"
    partial.write_text(json.dumps({"battery": {"capacity_mah": 3000}}))
    profile = load_profile(default_path=partial)
    assert profile.get("battery.capacity_mah") == 3000
    assert profile.sources["battery.capacity_mah"] == TIER_DEFAULT_FILE
    assert profile.sources["battery.cells_s"] == TIER_CONSTANTS


@pytest.mark.parametrize("payload, message", [
    ({"physical": {"mas_kg": 1.0}}, "unknown drone profile key"),
    ({"physical": {"mass_kg": "heavy"}}, "must be a number"),
    ({"physical": {"inertia_diag_kgm2": [1, 2]}}, "list of 3"),
    ({"physical": {"mass_kg": -1}}, "must be > 0"),
    ({"battery": {"cutoff_voltage_v": 20.0}}, "below nominal"),
])
def test_invalid_profiles_are_rejected(tmp_path, payload, message):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(payload))
    with pytest.raises(ValueError, match=message):
        load_profile(path)


def test_explicit_missing_profile_is_an_error(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_profile(tmp_path / "nope.json")
