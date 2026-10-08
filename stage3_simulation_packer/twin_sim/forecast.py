"""Ensemble weather forecast (Open-Meteo) as the weather of the Monte Carlo stress test.

One request fetches every member of an ensemble model for the show's site and day. Each member
becomes a `weather.Scenario` over the show window: mean wind at the show's height, turbulence
intensity from the gust factor, one gust front, rain, and the member's temperature.

Wind at the show height `h` comes from the levels the model returns (10, 80, 100, 120 m): linear in
ln z between two levels, a power law above the top level (exponent from the outermost two levels,
1/7 with 10 m only). The direction is the nearest level's.

Settings from the environment, or from `<repo>/.env` (KEY=value lines; a variable already set wins):
OPEN_METEO_API_KEY (customer key; none = the free, non-commercial API) and OPEN_METEO_MODEL.
"""

from __future__ import annotations

import json
import math
import os
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any

import numpy as np

from .profile import DroneProfile
from .weather import Scenario, rain_rule_defaults

ENV_FILE = Path(__file__).resolve().parents[2] / ".env"


def load_env_file(path: Path = ENV_FILE) -> None:
    """Put KEY=value lines of `path` into os.environ, without overriding variables already set.
    Blank lines and # comments are skipped; one pair of surrounding quotes is removed."""
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError:
        return
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = (part.strip() for part in line.removeprefix("export ").split("=", 1))
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
            value = value[1:-1]
        os.environ.setdefault(key, value)


load_env_file()

FREE_URL = "https://ensemble-api.open-meteo.com/v1/ensemble"
CUSTOMER_URL = "https://customer-ensemble-api.open-meteo.com/v1/ensemble"
DEFAULT_MODEL = os.environ.get("OPEN_METEO_MODEL") or "ecmwf_ifs025"
LEVELS_M = (10, 80, 100, 120)
TIMEOUT_S = 30.0
# Gust factor -> turbulence: G = U + g * sigma. A calibration constant, not a fixed truth.
PEAK_FACTOR = 3.0
TURBULENCE_MAX = 0.5
ALPHA_RANGE = (0.0, 0.4)
ALPHA_10M_ONLY = 1.0 / 7.0
MIN_SHOW_HEIGHT_M = 10.0
GUST_DURATION_RANGE_S = (2.0, 8.0)   # as the random stress test
GUST_SPREAD_RAD = math.pi / 6


class ForecastError(ValueError):
    pass


@dataclass
class ForecastSource:
    """Where the forecast comes from. `forecast_file` (a saved response) skips the request."""

    show_start: str                       # YYYY-MM-DDTHH:MM, local time at the site
    site: tuple[float, float] | None = None
    model: str = DEFAULT_MODEL
    api_key: str | None = None
    forecast_file: str | None = None
    save_to: str | None = None            # where a fetched response is saved
    peak_factor: float = PEAK_FACTOR


@dataclass
class Member:
    index: int                            # 0 = control run
    scenario: Scenario                    # gust list empty; drawn per flight
    ambient_c: float
    gust_excess_mps: float                # largest G_h - U_h over the window
    from_deg: float                       # wind direction at the show start
    wind_mean_mps: float
    wind_max_mps: float
    rain_max_mm_h: float


@dataclass
class Forecast:
    raw: dict[str, Any]
    source: ForecastSource
    fetched_at: str | None = None
    members: list[Member] = field(default_factory=list)
    show_height_m: float = 0.0
    levels_m: list[int] = field(default_factory=list)


# --------------------------------------------------------------------------- #
# Request
# --------------------------------------------------------------------------- #

def parse_show_start(text: str) -> datetime:
    try:
        return datetime.strptime(text, "%Y-%m-%dT%H:%M")
    except ValueError:
        raise ForecastError(f"show start {text!r}: use YYYY-MM-DDTHH:MM (local time at the site)") from None


def request_url(source: ForecastSource, duration_sec: float) -> str:
    if source.site is None:
        raise ForecastError("the forecast needs the show's site (latitude, longitude)")
    lat, lon = source.site
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise ForecastError(f"site {lat}, {lon}: latitude -90..90, longitude -180..180")
    start = parse_show_start(source.show_start)
    end = start + timedelta(seconds=duration_sec + 3600.0)   # the hour after the end holds its rain
    hourly = [f"wind_speed_{z}m" for z in LEVELS_M] + [f"wind_direction_{z}m" for z in LEVELS_M]
    params = {"latitude": f"{lat:.5f}", "longitude": f"{lon:.5f}", "models": source.model,
              "hourly": ",".join(hourly + ["wind_gusts_10m", "temperature_2m", "precipitation"]),
              "wind_speed_unit": "ms", "timezone": "auto",
              "start_date": start.date().isoformat(), "end_date": end.date().isoformat()}
    key = source.api_key or os.environ.get("OPEN_METEO_API_KEY")
    if key:
        params["apikey"] = key
    return f"{CUSTOMER_URL if key else FREE_URL}?{urllib.parse.urlencode(params)}"


def fetch(source: ForecastSource, duration_sec: float) -> dict[str, Any]:
    """The raw response (never contains the API key)."""
    url = request_url(source, duration_sec)
    try:
        with urllib.request.urlopen(url, timeout=TIMEOUT_S) as resp:
            return json.load(resp)
    except urllib.error.HTTPError as exc:
        try:
            reason = json.load(exc).get("reason", "")
        except (ValueError, AttributeError):
            reason = ""
        raise ForecastError(f"Open-Meteo answered HTTP {exc.code}{': ' + reason if reason else ''}") from None
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        raise ForecastError(f"Open-Meteo could not be reached: {getattr(exc, 'reason', exc)}") from None
    except ValueError:
        raise ForecastError("Open-Meteo sent a response that is not JSON") from None


def load(source: ForecastSource, duration_sec: float) -> tuple[dict[str, Any], str | None]:
    """(raw response, fetch time) from the saved file or the API."""
    if source.forecast_file:
        try:
            with open(source.forecast_file, encoding="utf-8") as fh:
                return json.load(fh), None
        except OSError as exc:
            raise ForecastError(f"forecast file: {exc}") from None
        except ValueError:
            raise ForecastError(f"forecast file {source.forecast_file}: not JSON") from None
    raw = fetch(source, duration_sec)
    if source.save_to:
        save_raw(raw, source.save_to)
    return raw, datetime.now().astimezone().isoformat(timespec="seconds")


# --------------------------------------------------------------------------- #
# Members
# --------------------------------------------------------------------------- #

def _defined(raw: dict[str, Any], var: str) -> bool:
    return var in raw.get("hourly", {}) and raw.get("hourly_units", {}).get(var) not in (None, "undefined")


def _series(raw: dict[str, Any], var: str, member: int) -> list | None:
    return raw["hourly"].get(var if member == 0 else f"{var}_member{member:02d}")


def member_count(raw: dict[str, Any]) -> int:
    hourly = raw.get("hourly", {})
    n = 1 if "wind_speed_10m" in hourly else 0
    while f"wind_speed_10m_member{n:02d}" in hourly:
        n += 1
    return n


def _at(hour_t: np.ndarray, values: list | None, t: np.ndarray) -> np.ndarray | None:
    """Values at times t (linear), or None when a value the window needs is missing."""
    if values is None:
        return None
    v = np.array([np.nan if x is None else x for x in values], float)
    lo, hi = np.searchsorted(hour_t, t.min(), side="right") - 1, np.searchsorted(hour_t, t.max(), side="left")
    if lo < 0 or hi >= hour_t.size or not np.all(np.isfinite(v[lo:hi + 1])):
        return None
    return np.interp(t, hour_t, v)


def wind_at_height(levels: dict[int, np.ndarray], h: float) -> np.ndarray:
    """Mean wind speed at height h from speeds at the given levels (each an array over time)."""
    zs = sorted(levels)
    if h <= zs[0]:
        return levels[zs[0]].copy()
    for z1, z2 in zip(zs, zs[1:]):
        if h <= z2:
            w = math.log(h / z1) / math.log(z2 / z1)
            return levels[z1] + (levels[z2] - levels[z1]) * w
    top = levels[zs[-1]]
    if len(zs) == 1:
        alpha = np.full(top.shape, ALPHA_10M_ONLY)
    else:
        low = levels[zs[0]]
        with np.errstate(divide="ignore", invalid="ignore"):
            alpha = np.log(top / low) / math.log(zs[-1] / zs[0])
        alpha = np.where(np.isfinite(alpha), alpha, ALPHA_10M_ONLY)
    return top * (h / zs[-1]) ** np.clip(alpha, *ALPHA_RANGE)


def build_members(raw: dict[str, Any], source: ForecastSource, profile: DroneProfile, duration_sec: float,
                  show_height_m: float) -> tuple[list[Member], list[int]]:
    """Every member as a scenario over [0, duration_sec] of show time; also the wind levels used."""
    if raw.get("error"):
        raise ForecastError(f"Open-Meteo: {raw.get('reason', 'error')}")
    if "hourly" not in raw or "time" not in raw["hourly"]:
        raise ForecastError("the forecast has no hourly data")
    for var in ("wind_speed_10m", "wind_direction_10m", "wind_gusts_10m", "temperature_2m", "precipitation"):
        if not _defined(raw, var):
            raise ForecastError(f"model {source.model!r} does not forecast {var}; choose another model")

    start = parse_show_start(source.show_start)
    hour_t = np.array([(datetime.fromisoformat(s) - start).total_seconds() for s in raw["hourly"]["time"]])
    rain_t = hour_t - 1800.0      # an hour's sum is the rate at the middle of that hour
    inside = lambda t: t[(t > 0.0) & (t < duration_sec)]  # noqa: E731
    keys = np.unique(np.concatenate([[0.0, duration_sec], inside(hour_t), inside(rain_t)]))
    rule = rain_rule_defaults(profile)
    levels_used = [z for z in LEVELS_M if _defined(raw, f"wind_speed_{z}m") and _defined(raw, f"wind_direction_{z}m")]

    members: list[Member] = []
    levels_m: list[int] = []
    for m in range(member_count(raw)):
        speed, direction = {}, {}
        for z in levels_used:
            s = _at(hour_t, _series(raw, f"wind_speed_{z}m", m), keys)
            d = _at(hour_t, _series(raw, f"wind_direction_{z}m", m), keys)
            if s is not None and d is not None:
                speed[z], direction[z] = np.maximum(s, 0.0), d
        gust10 = _at(hour_t, _series(raw, "wind_gusts_10m", m), keys)
        temp = _at(hour_t, _series(raw, "temperature_2m", m), keys)
        rain = _at(rain_t, _series(raw, "precipitation", m), keys)
        if 10 not in speed or gust10 is None or temp is None or rain is None:
            if m == 0:
                first, last = raw["hourly"]["time"][0], raw["hourly"]["time"][-1]
                raise ForecastError(f"the show window starting {source.show_start} is outside the forecast "
                                    f"({first} .. {last}) or has missing values")
            continue
        if m == 0:
            levels_m = sorted(speed)
        u_h = wind_at_height(speed, show_height_m)
        nearest = min(direction, key=lambda z: abs(z - show_height_m))
        u10 = speed[10]
        gust_factor = np.where(u10 > 0.1, np.maximum(gust10, u10) / np.maximum(u10, 0.1), 1.0)
        turb = np.clip((gust_factor - 1.0) / source.peak_factor, 0.0, TURBULENCE_MAX)
        scenario = Scenario(
            name=f"{source.model} member {m}", seed=0,
            wind=[{"t": float(t), "speed_mps": float(u), "from_deg": float(d) % 360.0, "turbulence": float(ti)}
                  for t, u, d, ti in zip(keys, u_h, direction[nearest], turb)],
            rtk=[{"t": 0.0, "state": "fixed"}],
            rain=[{"t": float(t), "mm_h": float(max(r, 0.0))} for t, r in zip(keys, rain)],
            rain_rule=dict(rule),
        )
        members.append(Member(index=m, scenario=scenario, ambient_c=float(temp[0]),
                              gust_excess_mps=float(np.max((gust_factor - 1.0) * u_h)),
                              from_deg=float(direction[nearest][0]) % 360.0,
                              wind_mean_mps=float(np.mean(u_h)), wind_max_mps=float(np.max(u_h)),
                              rain_max_mm_h=float(max(np.max(rain), 0.0))))
    return members, levels_m


def show_height(pw, samples: int = 64) -> float:
    """Mean planned height above z = 0 over the show, at least MIN_SHOW_HEIGHT_M."""
    from .loaders.spline_evaluator import evaluate_numpy

    pos, _, _ = evaluate_numpy(pw, np.linspace(pw.start_time_sec, pw.end_time_sec, samples))
    return max(MIN_SHOW_HEIGHT_M, float(np.mean(pos[..., 2])))


def prepare(source: ForecastSource, profile: DroneProfile, pw, duration_sec: float) -> Forecast:
    raw, fetched_at = load(source, duration_sec)
    height = show_height(pw)
    members, levels = build_members(raw, source, profile, duration_sec, height)
    return Forecast(raw=raw, source=source, fetched_at=fetched_at, members=members, show_height_m=height,
                    levels_m=levels)


def flight_scenario(member: Member, seed: int, duration_sec: float) -> Scenario:
    """The member's scenario with this flight's seed and one random gust front."""
    rng = np.random.default_rng([seed, 1])
    gust = {"t": float(rng.uniform(0.0, max(duration_sec, 1.0))),
            "peak_mps": float(rng.uniform(0.0, member.gust_excess_mps)),
            "duration_s": float(rng.uniform(*GUST_DURATION_RANGE_S)),
            "from_deg": (member.from_deg + math.degrees(rng.uniform(-GUST_SPREAD_RAD, GUST_SPREAD_RAD))) % 360.0}
    s = member.scenario
    return Scenario(name=s.name, seed=seed, wind=s.wind, gusts=[gust], rtk=s.rtk, rain=s.rain,
                    rain_rule=s.rain_rule)


def save_raw(raw: dict[str, Any], path: str | Path) -> None:
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(raw, fh)
