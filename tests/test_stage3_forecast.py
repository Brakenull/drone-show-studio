import json
import math

import pytest

from stage3_helpers import grid_show, opencl_available

from stage3_simulation_packer.twin_sim import forecast as fc
from stage3_simulation_packer.twin_sim.profile import load_profile

LEVELS = (10, 80, 100, 120)


def synthetic_forecast(members: int = 3, drop: tuple[str, ...] = ()) -> dict:
    """2026-10-20 00:00 .. 2026-10-21 23:00; 10 m = 4 m/s, 100 m = 6 m/s, gusts 7 m/s, from 270 deg;
    temperature 28 C at 19:00, 30 C after; 2 mm of rain in the hour 20:00-21:00."""
    times = [f"2026-10-{20 + h // 24}T{h % 24:02d}:00" for h in range(48)]
    series = {
        "wind_speed_10m": [4.0] * 48, "wind_direction_10m": [270.0] * 48,
        "wind_speed_100m": [6.0] * 48, "wind_direction_100m": [280.0] * 48,
        "wind_gusts_10m": [7.0] * 48,
        "temperature_2m": [28.0 if h <= 19 else 30.0 for h in range(48)],
        "precipitation": [2.0 if h == 21 else 0.0 for h in range(48)],
    }
    hourly, units = {"time": times}, {"time": "iso8601"}
    for var in [f"wind_speed_{z}m" for z in LEVELS] + [f"wind_direction_{z}m" for z in LEVELS] + [
            "wind_gusts_10m", "temperature_2m", "precipitation"]:
        values = series.get(var) if var not in drop else None
        for m in range(members):
            name = var if m == 0 else f"{var}_member{m:02d}"
            hourly[name] = values if values is not None else [None] * 48
            units[name] = "undefined" if values is None else "m/s"
    return {"latitude": 10.75, "longitude": 106.75, "hourly_units": units, "hourly": hourly}


def source(**kw):
    return fc.ForecastSource(show_start=kw.pop("show_start", "2026-10-20T19:30"), site=(10.76, 106.66), **kw)


def test_wind_at_height_log_between_levels_and_power_law_above():
    levels = {10: fc.np.array([4.0]), 100: fc.np.array([6.0])}
    assert fc.wind_at_height(levels, 50.0)[0] == pytest.approx(4.0 + 2.0 * math.log(5.0) / math.log(10.0))
    alpha = math.log(6.0 / 4.0) / math.log(10.0)
    assert fc.wind_at_height(levels, 150.0)[0] == pytest.approx(6.0 * 1.5 ** alpha)
    assert fc.wind_at_height({10: fc.np.array([4.0])}, 40.0)[0] == pytest.approx(4.0 * 4.0 ** (1 / 7))


def test_members_match_hand_values():
    members, levels = fc.build_members(synthetic_forecast(), source(), load_profile(), 5400.0, 50.0)
    assert levels == [10, 100] and [m.index for m in members] == [0, 1, 2]
    m = members[0]
    u_h = 4.0 + 2.0 * math.log(5.0) / math.log(10.0)
    assert [k["t"] for k in m.scenario.wind] == [0.0, 1800.0, 3600.0, 5400.0]
    assert m.scenario.wind[0]["speed_mps"] == pytest.approx(u_h)
    assert m.scenario.wind[0]["from_deg"] == 270.0                    # 10 m is nearest to 50 m
    assert m.scenario.wind[0]["turbulence"] == pytest.approx((7.0 / 4.0 - 1.0) / 3.0)
    assert m.gust_excess_mps == pytest.approx(0.75 * u_h)
    assert m.ambient_c == pytest.approx(29.0)                          # 19:30
    assert m.rain_max_mm_h == pytest.approx(2.0)                       # the 21:00 sum, centred at 20:30
    assert [k["mm_h"] for k in m.scenario.rain] == pytest.approx([0.0, 1.0, 2.0, 1.0])


def test_flight_scenario_is_seeded_and_bounded():
    m = fc.build_members(synthetic_forecast(), source(), load_profile(), 600.0, 50.0)[0][0]
    a, b, c = fc.flight_scenario(m, 11, 600.0), fc.flight_scenario(m, 11, 600.0), fc.flight_scenario(m, 12, 600.0)
    assert a.gusts == b.gusts and a.gusts != c.gusts and a.seed == 11
    g = a.gusts[0]
    assert 0.0 <= g["peak_mps"] <= m.gust_excess_mps and 0.0 <= g["t"] <= 600.0
    assert abs((g["from_deg"] - m.from_deg + 180.0) % 360.0 - 180.0) <= 30.0


def test_bad_forecasts_are_reported():
    profile = load_profile()
    with pytest.raises(fc.ForecastError, match="wind_gusts_10m"):
        fc.build_members(synthetic_forecast(drop=("wind_gusts_10m",)), source(), profile, 600.0, 50.0)
    with pytest.raises(fc.ForecastError, match="outside the forecast"):
        fc.build_members(synthetic_forecast(), source(show_start="2026-10-25T19:30"), profile, 600.0, 50.0)
    with pytest.raises(fc.ForecastError, match="Open-Meteo: bad"):
        fc.build_members({"error": True, "reason": "bad"}, source(), profile, 600.0, 50.0)
    with pytest.raises(fc.ForecastError, match="YYYY"):
        fc.parse_show_start("20/10/2026")


def test_request_url_uses_customer_endpoint_only_with_key(monkeypatch):
    monkeypatch.delenv("OPEN_METEO_API_KEY", raising=False)
    free = fc.request_url(source(), 600.0)
    assert free.startswith(fc.FREE_URL) and "apikey" not in free and "wind_speed_unit=ms" in free
    paid = fc.request_url(source(api_key="K"), 600.0)
    assert paid.startswith(fc.CUSTOMER_URL) and "apikey=K" in paid


@pytest.mark.skipif(not opencl_available(), reason="no OpenCL device")
def test_forecast_monte_carlo_flies_members_reproducibly(tmp_path):
    pytest.importorskip("pyopencl")
    from stage3_simulation_packer.twin_sim import monte_carlo_runner as mc

    path = tmp_path / "forecast.json"
    path.write_text(json.dumps(synthetic_forecast()), encoding="utf-8")
    src = fc.ForecastSource(show_start="2026-10-20T20:55", forecast_file=str(path))
    show = grid_show(2, spacing=2.5, climb=6.0, duration=4.0, hold=2.0)
    cfg = mc.StressConfig(runs=4, tail_sec=0.5)
    a = mc.run_monte_carlo(show, cfg=cfg, forecast=src)
    b = mc.run_monte_carlo(show, cfg=cfg, forecast=src)
    assert [r["member"] for r in a["runs"]] == [0, 1, 2, 0]
    assert [r["min_separation_m"] for r in a["runs"]] == [r["min_separation_m"] for r in b["runs"]]
    ws = a["weather_source"]
    assert ws["members"] == 3 and ws["levels_m"] == [10, 100] and ws["show_height_m"] == 10.0
    assert a["runs"][0]["scenario"]["ambient_c"] == 30.0
    assert a["summary"]["rain_alert_members"] == 1.0 and a["summary"]["rain_alert_runs"] == [0, 1, 2, 3]


def test_env_file_fills_unset_variables_only(tmp_path, monkeypatch):
    env = tmp_path / ".env"
    env.write_text('# comment\n\nFC_A=plain\nexport FC_B = "quoted value"\nFC_C=\'x\'\nFC_SET=from-file\nnot a pair\n',
                   encoding="utf-8")
    for k in ("FC_A", "FC_B", "FC_C"):
        monkeypatch.delenv(k, raising=False)
    monkeypatch.setenv("FC_SET", "from-env")
    fc.load_env_file(env)
    import os
    assert (os.environ["FC_A"], os.environ["FC_B"], os.environ["FC_C"]) == ("plain", "quoted value", "x")
    assert os.environ["FC_SET"] == "from-env"
    fc.load_env_file(tmp_path / "missing.env")  # no file: nothing happens
