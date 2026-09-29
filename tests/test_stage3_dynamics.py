"""Digital twin physics checks (default OpenCL device; short shows to keep the suite fast)."""

import numpy as np
import pytest

from stage3_helpers import grid_show, line_control_points, make_contract, make_segment, opencl_available
from stage3_simulation_packer.twin_sim.loaders.arrow_loader import load_trajectories  # noqa: E402
from stage3_simulation_packer.twin_sim.profile import load_profile  # noqa: E402

pytestmark = pytest.mark.skipif(not opencl_available(), reason="no OpenCL device")

if opencl_available():
    from stage3_simulation_packer.twin_sim.simulator import DigitalTwin, Disturbances, SimConfig


@pytest.fixture(scope="module")
def profile():
    return load_profile()


def hover_show(positions, climb_s=6.0, hold_s=6.0):
    drones = []
    for x, y, z in positions:
        drones.append([
            make_segment(0, 0.0, climb_s, line_control_points([x, y, 0], [x, y, z], 8)),
            make_segment(1, climb_s, climb_s + hold_s, line_control_points([x, y, z], [x, y, z], 6)),
        ])
    return make_contract(drones)


def test_nominal_climb_and_hover_tracks_reference(profile):
    show = load_trajectories(grid_show(2, duration=6.0, hold=4.0))
    twin = DigitalTwin(show, profile)
    res = twin.run(Disturbances.nominal(profile, 4), SimConfig())
    assert np.all(res.max_tracking_error_m < 0.05)
    np.testing.assert_allclose(res.final_position[:, 2], 10.0, atol=0.02)
    assert np.all(res.min_separation_m == pytest.approx(2.0, abs=0.02))
    assert not res.brownout.any()
    # ~10 s of flight at ~5 A from 2.2 Ah: roughly 0.5-1 % SOC
    assert np.all((0.985 < res.final_soc) & (res.final_soc < 0.998))


def test_parked_drones_stay_disarmed_and_only_draw_idle_current(profile):
    contract = make_contract([[make_segment(0, 0.0, 10.0, [[0, 0, 0]] * 6, colors=[(0.0, [0, 0, 0])])]])
    twin = DigitalTwin(load_trajectories(contract), profile)
    res = twin.run(Disturbances.nominal(profile, 1), SimConfig())
    np.testing.assert_allclose(res.final_position[0], [0, 0, 0], atol=1e-6)
    drained = 1.0 - res.final_soc[0]
    avionics_only = profile.get("battery.avionics_current_a") * 10.0 / 3600.0 / 2.2
    assert drained == pytest.approx(avionics_only, rel=0.1)


def test_downwash_makes_the_lower_drone_sag(profile):
    # Lower drone flies under an upper one 1.2 m above, then hovers there.
    def show(with_upper: bool):
        low = [make_segment(0, 0.0, 6.0, line_control_points([0, 0, 0], [0, 0, 10], 8)),
               make_segment(1, 6.0, 14.0, line_control_points([0, 0, 10], [0, 0, 10], 6))]
        upper_z = 11.2 if with_upper else 30.0
        up = [make_segment(0, 0.0, 6.0, line_control_points([0, 0, 0.0], [0, 0, upper_z], 8)),
              make_segment(1, 6.0, 14.0, line_control_points([0, 0, upper_z], [0, 0, upper_z], 6))]
        # start them apart on the pad; both climb vertically so the stack forms in the air
        low[0]["control_points"] = line_control_points([3, 0, 0], [0, 0, 10], 8).tolist()
        return make_contract([low, up])

    errors = []
    for with_upper in (False, True):
        twin = DigitalTwin(load_trajectories(show(with_upper)), profile)
        res = twin.run(Disturbances.nominal(profile, 2), SimConfig(record_hz=20))
        z_low = res.recorded_positions[:, 0, 2]
        hover = res.recorded_times > 7.0
        errors.append(np.max(10.0 - z_low[hover]))
    assert errors[1] > errors[0] + 0.05      # measurable lift loss from the wake
    assert errors[1] < 0.8                   # but the controller holds it


def test_wind_increases_tracking_error_but_stays_controlled(profile):
    show = load_trajectories(hover_show([(0, 0, 10)]))
    twin = DigitalTwin(show, profile)
    calm = twin.run(Disturbances.nominal(profile, 1), SimConfig())
    windy_dist = Disturbances.nominal(profile, 1)
    windy_dist.mean_wind = np.array([9.0, 0.0, 0.0])
    windy_dist.gust_vec = np.array([6.0, 0.0, 0.0])
    windy_dist.gust_start, windy_dist.gust_duration, windy_dist.gust_speed = 7.0, 3.0, 9.0
    windy = twin.run(windy_dist, SimConfig())
    assert windy.max_tracking_error_m[0] > calm.max_tracking_error_m[0] + 0.1
    assert windy.max_tracking_error_m[0] < 1.5
    assert windy.final_soc[0] < calm.final_soc[0]     # fighting wind costs energy


def test_close_approach_is_detected_as_crash_pair(profile):
    # Two drones planned to pass 0.3 m apart at t = 5 s.
    a = [make_segment(0, 0.0, 10.0, line_control_points([-10, 0.15, 10], [10, 0.15, 10], 8))]
    b = [make_segment(0, 0.0, 10.0, line_control_points([10, -0.15, 10], [-10, -0.15, 10], 8))]
    twin = DigitalTwin(load_trajectories(make_contract([a, b])), profile)
    res = twin.run(Disturbances.nominal(profile, 2), SimConfig())
    assert res.min_separation_m.min() < 0.5
    assert res.min_separation_partner.tolist() == [1, 0]
    assert 4.0 < res.min_separation_time[0] < 6.0


def test_depleted_battery_raises_brownout(profile):
    show = load_trajectories(hover_show([(0, 0, 10)], climb_s=4.0, hold_s=8.0))
    twin = DigitalTwin(show, profile)
    dist = Disturbances.nominal(profile, 1)
    dist.initial_soc = np.array([0.03])
    res = twin.run(dist, SimConfig())
    assert res.brownout[0]
    assert res.min_voltage_v[0] <= profile.get("battery.cutoff_voltage_v")
    assert res.final_soc[0] < 0.03


def test_cold_battery_sags_more(profile):
    show = load_trajectories(hover_show([(0, 0, 10)], climb_s=4.0, hold_s=4.0))
    twin = DigitalTwin(show, profile)
    warm = twin.run(Disturbances.nominal(profile, 1, ambient_c=25.0), SimConfig())
    cold = twin.run(Disturbances.nominal(profile, 1, ambient_c=-5.0), SimConfig())
    assert cold.min_voltage_v[0] < warm.min_voltage_v[0] - 0.05


def test_show_starting_airborne_is_not_treated_as_ground(profile):
    # Regression: the ground plane used to be inferred from the lowest start
    # altitude, so a show starting at z = 5 m "slid along the ground" there.
    seg = make_segment(0, 0.0, 6.0, line_control_points([-6, 0, 5], [6, 0, 5], 8))
    twin = DigitalTwin(load_trajectories(make_contract([[seg]])), profile)
    assert twin.ground_z == 0.0
    res = twin.run(Disturbances.nominal(profile, 1), SimConfig(tail_sec=1.0))
    assert res.max_tracking_error_m[0] < 0.15
    np.testing.assert_allclose(res.final_position[0], [6, 0, 5], atol=0.05)
