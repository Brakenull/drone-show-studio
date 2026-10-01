"""Condition simulator, milestone C1 (docs/4-condition_simulator.md §3, B6, B8, §9).

Weather timeline interpolation (§9.1), compatibility of a constant timeline with the Monte Carlo
path (§9.2), recording for playback, and determinism (§9.8).
"""

import copy
import json
import math

import numpy as np
import pytest

from stage3_helpers import grid_show, line_control_points, make_contract, make_segment, opencl_available
from stage3_simulation_packer.twin_sim import weather
from stage3_simulation_packer.twin_sim.profile import load_profile
from stage3_simulation_packer.twin_sim.weather import Scenario, ScenarioError, validate_scenario

needs_opencl = pytest.mark.skipif(not opencl_available(), reason="no OpenCL device")


def scenario(**overrides) -> Scenario:
    data = {"name": "test", "seed": 7,
            "wind": [{"t": 0, "speed_mps": 3.0, "from_deg": 350, "turbulence": 0.1},
                     {"t": 10, "speed_mps": 9.0, "from_deg": 10, "turbulence": 0.3}],
            "gusts": [], "rtk": [{"t": 0, "state": "fixed"}, {"t": 5, "state": "float"}, {"t": 8, "state": "gps"}],
            "rain": [{"t": 0, "mm_h": 0.0}, {"t": 4, "mm_h": 0.0}, {"t": 8, "mm_h": 6.0}]}
    data.update(overrides)
    return Scenario.from_dict(data)


# --------------------------------------------------------------------------- #
# §9.1 interpolation
# --------------------------------------------------------------------------- #

def test_wind_interpolates_linearly_and_turns_the_short_way_round():
    s = scenario()
    speed, direction, turb = weather.wind_at(s, [0.0, 5.0, 10.0, 2.5])
    np.testing.assert_allclose(speed, [3.0, 6.0, 9.0, 4.5])
    np.testing.assert_allclose(turb, [0.1, 0.2, 0.3, 0.15])
    # 350 -> 10 passes through north (0), not through 180.
    np.testing.assert_allclose(direction, [350.0, 0.0, 10.0, 355.0], atol=1e-9)
    s = scenario(wind=[{"t": 0, "speed_mps": 1, "from_deg": 10, "turbulence": 0},
                       {"t": 2, "speed_mps": 1, "from_deg": 300, "turbulence": 0}])
    assert float(weather.wind_at(s, 1.0)[1]) == pytest.approx(335.0)


def test_channels_hold_their_first_and_last_values_outside_the_keys():
    s = scenario(wind=[{"t": 4, "speed_mps": 2, "from_deg": 90, "turbulence": 0.2},
                       {"t": 6, "speed_mps": 4, "from_deg": 90, "turbulence": 0.2}])
    speed, _, _ = weather.wind_at(s, [0.0, 100.0])
    np.testing.assert_allclose(speed, [2.0, 4.0])
    np.testing.assert_allclose(weather.rain_at(s, [-1.0, 2.0, 6.0, 50.0]), [0.0, 0.0, 3.0, 6.0])


def test_rtk_changes_in_steps():
    s = scenario()
    assert [weather.rtk_state_at(s, t) for t in (0, 4.99, 5.0, 7.9, 8.0, 99)] == \
        ["fixed", "fixed", "float", "float", "gps", "gps"]


def test_wind_direction_is_where_it_blows_from_in_enu():
    # From the west (270): the air moves east (+x). From the north (0): it moves south (-y).
    np.testing.assert_allclose(weather.toward_unit(270.0), [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(weather.toward_unit(0.0), [0.0, -1.0, 0.0], atol=1e-12)


def test_table_columns():
    profile = load_profile()
    s = scenario(wind=[{"t": 0, "speed_mps": 4.0, "from_deg": 270, "turbulence": 0.25}])
    table = weather.tabulate(s, 10.0, profile)
    assert table.hz == weather.TABLE_HZ and table.rows.shape[0] >= 101
    np.testing.assert_allclose(table.rows[:, weather.WX_WIND:weather.WX_WIND + 3], [[4.0, 0.0, 0.0]] * table.rows.shape[0],
                               atol=1e-12)
    np.testing.assert_allclose(table.rows[:, weather.WX_TURB], 1.0)                       # 0.25 x 4 m/s
    np.testing.assert_allclose(table.rows[:, weather.WX_ADVECT], 4.0 * table.times)      # carried at 4 m/s
    noise = table.rows[:, weather.WX_GNSS_NOISE]
    t = table.times
    assert np.all(noise[t < 5] == profile.get("tolerances.gnss_rtk_noise_m"))
    assert np.all(noise[(t >= 5) & (t < 8)] == 0.3) and np.all(noise[t >= 8] == 1.5)
    # Calm air keeps the stress test's turbulence floor.
    calm = weather.tabulate(scenario(wind=[]), 1.0, profile)
    np.testing.assert_allclose(calm.rows[:, weather.WX_TURB], weather.TURBULENCE_FLOOR_MPS)


def test_validation_lists_every_problem():
    errors = validate_scenario({"name": " ", "seed": -1,
                                "wind": [{"t": -1, "speed_mps": 99, "from_deg": "w", "turbulence": 0.1}],
                                "rtk": [{"t": 0, "state": "rtk"}],
                                "rain_rule": {"alert_mm_h": 3, "limit_mm_h": 2}})
    joined = " | ".join(errors)
    for fragment in ("name", "seed", "wind[0].t", "wind[0].speed_mps", "wind[0].from_deg", "rtk[0].state",
                     "limit level must be above"):
        assert fragment in joined
    with pytest.raises(ScenarioError):
        Scenario.from_dict({"name": "x", "gusts": [{"t": 1}]})
    assert Scenario.from_dict({"name": "x"}).rain_rule == weather.DEFAULT_RAIN_RULE


def test_rain_crossing_is_exact():
    from stage3_simulation_packer.twin_sim.scenario_runner import rain_crossing

    s = scenario()
    assert rain_crossing(s, 0.5) == pytest.approx(4.0 + 4.0 * 0.5 / 6.0)
    assert rain_crossing(s, 6.0) == pytest.approx(8.0)
    assert rain_crossing(s, 7.0) is None


def test_gust_reaches_the_field_centre_at_its_time():
    s = scenario(gusts=[{"t": 20, "peak_mps": 5, "duration_s": 4, "from_deg": 270}])
    [row] = weather.gust_fronts(s, (30.0, -10.0))
    np.testing.assert_allclose(row[0:3], [5.0, 0.0, 0.0], atol=1e-12)
    speed = row[8]
    assert speed == pytest.approx(9.0)          # the wind speed at t = 20
    # Leading edge at the centre: local time 0 there exactly at t = 20.
    assert 20.0 - row[6] - 30.0 / speed == pytest.approx(0.0)


# --------------------------------------------------------------------------- #
# §9.2 compatibility: a constant timeline reproduces the Monte Carlo path
# --------------------------------------------------------------------------- #

@needs_opencl
def test_constant_timeline_reproduces_the_constant_weather_run():
    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import load_trajectories
    from stage3_simulation_packer.twin_sim.simulator import DigitalTwin, SimConfig

    profile = load_profile()
    twin = DigitalTwin(load_trajectories(grid_show(3, spacing=1.6, duration=8.0, hold=6.0)), profile)
    s = scenario(wind=[{"t": 0, "speed_mps": 6.0, "from_deg": 240, "turbulence": 0.2}],
                 gusts=[{"t": 9, "peak_mps": 4.0, "duration_s": 3.0, "from_deg": 250}],
                 rtk=[{"t": 0, "state": "fixed"}], rain=[])
    timeline = weather.scenario_disturbances(s, profile, twin.n, 15.0, (2.0, 2.0))

    # The same flight written the Monte Carlo way: constant values, modes scaled and moving with time.
    sigma = max(weather.TURBULENCE_FLOOR_MPS, 0.2 * 6.0)
    constant = copy.copy(timeline)
    constant.weather = None
    constant.mean_wind = weather.toward_unit(240.0) * 6.0
    constant.mode_amp = timeline.mode_amp * sigma
    constant.mode_omega = timeline.mode_omega * 6.0
    [gust] = timeline.gust_rows()
    constant.gusts = None
    constant.gust_vec, constant.gust_dir = gust[0:3], gust[3:6]
    constant.gust_start, constant.gust_duration, constant.gust_speed = gust[6], gust[7], gust[8]

    a = twin.run(timeline, SimConfig(tail_sec=1.0))
    b = twin.run(constant, SimConfig(tail_sec=1.0))
    np.testing.assert_allclose(a.final_position, b.final_position, atol=2e-3)
    np.testing.assert_allclose(a.max_tracking_error_m, b.max_tracking_error_m, atol=2e-3)
    np.testing.assert_allclose(a.min_separation_m, b.min_separation_m, atol=2e-3)
    np.testing.assert_allclose(a.final_soc, b.final_soc, atol=1e-5)
    assert a.max_tracking_error_m.max() > 0.02          # the weather did act


# --------------------------------------------------------------------------- #
# Time-varying weather acts when it should
# --------------------------------------------------------------------------- #

def hover_contract(hold_s: float = 14.0) -> dict:
    drones = []
    for x in (0.0, 4.0):
        drones.append([make_segment(0, 0.0, 6.0, line_control_points([x, 0, 0], [x, 0, 10], 8)),
                       make_segment(1, 6.0, 6.0 + hold_s, line_control_points([x, 0, 10], [x, 0, 10], 6))])
    return make_contract(drones)


@needs_opencl
def test_wind_and_rtk_changes_take_effect_at_their_time():
    from stage3_simulation_packer.twin_sim.scenario_runner import fly_scenario

    calm_then_windy = scenario(wind=[{"t": 0, "speed_mps": 0.0, "from_deg": 270, "turbulence": 0.0},
                                     {"t": 12, "speed_mps": 0.0, "from_deg": 270, "turbulence": 0.0},
                                     {"t": 12.5, "speed_mps": 12.0, "from_deg": 270, "turbulence": 0.0}],
                               rtk=[{"t": 0, "state": "fixed"}], rain=[])
    flight = fly_scenario(hover_contract(), calm_then_windy)
    dev = np.linalg.norm(flight.positions - flight.reference, axis=2).max(axis=1)
    t = flight.times
    before = dev[(t > 8) & (t < 12)].max()
    after = dev[(t > 12.5) & (t < 16)].max()
    assert before < 0.15 and after > before + 0.1
    # Pushed downwind (east, +x) when the wind arrives.
    drift = (flight.positions - flight.reference)[(t > 12.5) & (t < 14), :, 0]
    assert drift.mean() > 0.0

    gps_late = scenario(wind=[], rtk=[{"t": 0, "state": "fixed"}, {"t": 12, "state": "gps"}], rain=[])
    flight = fly_scenario(hover_contract(), gps_late)
    dev = np.linalg.norm(flight.positions - flight.reference, axis=2).max(axis=1)
    assert dev[(flight.times > 8) & (flight.times < 12)].max() < 0.15
    assert dev[flight.times > 14].max() > 0.3


# --------------------------------------------------------------------------- #
# B8 recording and §9.8 determinism
# --------------------------------------------------------------------------- #

@needs_opencl
def test_recording_and_determinism():
    from stage3_simulation_packer.twin_sim.loaders.arrow_loader import load_trajectories
    from stage3_simulation_packer.twin_sim.loaders.spline_evaluator import build_piecewise, evaluate_numpy
    from stage3_simulation_packer.twin_sim.scenario_runner import RECORD_HZ, TAIL_SEC, fly_scenario

    contract = grid_show(2, duration=6.0, hold=3.0)
    s = scenario(gusts=[{"t": 5, "peak_mps": 3.0, "duration_s": 2.0, "from_deg": 0}])
    one = fly_scenario(contract, s)
    two = fly_scenario(contract, s)

    show = 9.0
    assert one.times[0] == 0.0 and one.times[-1] == pytest.approx(show + TAIL_SEC)
    np.testing.assert_allclose(np.diff(one.times), 1.0 / RECORD_HZ, atol=1e-9)
    n = 4
    assert one.positions.shape == one.reference.shape == one.colors.shape == (one.times.size, n, 3)
    pos, _, _ = evaluate_numpy(build_piecewise(load_trajectories(contract)), one.times)
    np.testing.assert_allclose(one.reference, pos.transpose(1, 0, 2), atol=1e-4)
    np.testing.assert_allclose(one.positions[0], one.reference[0], atol=0.2)  # launch placement error only

    timing = ("wall_time_sec", "realtime_factor")
    strip = lambda r: {k: v for k, v in r.items() if k not in timing}  # noqa: E731
    assert json.dumps(strip(one.report), sort_keys=True) == json.dumps(strip(two.report), sort_keys=True)
    assert np.array_equal(one.positions, two.positions)
    assert math.isfinite(one.report["largest_deviation"]["distance_m"])
