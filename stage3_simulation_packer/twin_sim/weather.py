"""Weather timeline of a condition-simulator scenario.

A scenario is a weather timeline over show time (t = 0 at the show start):

* wind keys   -- mean speed at the reference height, direction it blows *from*
                 (meteorological: 0 = north, 90 = east), turbulence intensity
                 sigma / mean speed; linear between keys, direction the short
                 way round;
* gusts       -- discrete (1 - cos) gust fronts, the Stage 3 shape, sweeping
                 across the field at the wind speed of the event's time;
* RTK keys    -- `fixed` / `float` / `gps`, a step at each key;
* rain keys   -- mm/h, linear; no physical effect in this revision.

Every channel holds its first value before its first key and its last value
after its last key. `tabulate()` turns the timeline into the 10 Hz table the
kernels interpolate (`aerodynamics.cl` `wind_at`, `multi_agent_dynamics.cl`
GNSS), and `scenario_disturbances()` builds the `Disturbances` of one flight:
turbulence modes, gusts and per-drone hardware spread all drawn from the
scenario's seed, so a scenario always simulates the same way.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .profile import DroneProfile

TABLE_HZ = 10.0
# Columns of one table row (kernels/common.cl WX_*).
WX_WIND = 0          # mean wind x, y, z (m/s, ENU)
WX_TURB = 3          # turbulence sigma (m/s); the modes have unit amplitude
WX_ADVECT = 4        # distance the turbulence has been carried, integral of max(speed, 1) dt (m)
WX_GNSS_NOISE = 5    # m
WX_GNSS_DRIFT = 6    # m / sqrt(s)
WX_RAIN = 7          # mm/h (not used by the kernels)
WX_STRIDE = 8

RTK_STATES = ("fixed", "float", "gps")
# (noise m, drift m/sqrt(s)) for the degraded states; `fixed` takes the profile's tolerances.
# Defaults to be confirmed.
RTK_DEGRADED = {"float": (0.3, 0.05), "gps": (1.5, 0.2)}

# Same turbulence and gust model as the Monte Carlo stress test (monte_carlo_runner.StressConfig).
TURBULENCE_FLOOR_MPS = 0.3
TURBULENCE_MODES = 8
GUST_MIN_SPEED_MPS = 2.0
AMBIENT_C = 25.0

# Rain rule: alert, limit and reaction come from the drone profile's `environment` keys
# (placeholders until the drone's water protection rating is known); the margin is a planning choice.
DEFAULT_RAIN_RULE = {"alert_mm_h": 0.5, "limit_mm_h": 2.5, "reaction_s": 5.0, "margin_s": 10.0}


def rain_rule_defaults(profile: DroneProfile) -> dict[str, float]:
    return {"alert_mm_h": float(profile.get("environment.rain_alert_mm_h")),
            "limit_mm_h": float(profile.get("environment.rain_limit_mm_h")),
            "reaction_s": float(profile.get("environment.return_reaction_s")),
            "margin_s": DEFAULT_RAIN_RULE["margin_s"]}

# Meteorological rain scale, upper bounds in mm/h.
RAIN_SCALE = (("drizzle", 0.5), ("light", 2.5), ("moderate", 7.6), ("heavy", math.inf))

LIMITS = {"speed_mps": (0.0, 40.0), "turbulence": (0.0, 1.0), "peak_mps": (0.0, 30.0),
          "duration_s": (0.1, 60.0), "mm_h": (0.0, 200.0)}


class ScenarioError(ValueError):
    def __init__(self, errors: list[str]):
        super().__init__("; ".join(errors))
        self.errors = errors


@dataclass
class Scenario:
    name: str
    seed: int
    wind: list[dict[str, float]] = field(default_factory=list)
    gusts: list[dict[str, float]] = field(default_factory=list)
    rtk: list[dict[str, Any]] = field(default_factory=list)
    rain: list[dict[str, float]] = field(default_factory=list)
    rain_rule: dict[str, float] = field(default_factory=lambda: dict(DEFAULT_RAIN_RULE))

    @classmethod
    def from_dict(cls, data: Any, rule_defaults: dict[str, float] | None = None) -> "Scenario":
        """Validated scenario; keys are sorted by time. Raises ScenarioError listing every problem.
        Rain-rule values the scenario leaves out come from `rule_defaults` (the drone profile's)."""
        errors = validate_scenario(data)
        if errors:
            raise ScenarioError(errors)
        by_t = lambda keys: sorted((dict(k) for k in keys), key=lambda k: k["t"])  # noqa: E731
        return cls(
            name=str(data["name"]).strip(),
            seed=int(data.get("seed", 0)),
            wind=by_t(data.get("wind", [])),
            gusts=by_t(data.get("gusts", [])),
            rtk=by_t(data.get("rtk", [])),
            rain=by_t(data.get("rain", [])),
            rain_rule={**(rule_defaults or DEFAULT_RAIN_RULE), **(data.get("rain_rule") or {})},
        )

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "seed": self.seed, "wind": self.wind, "gusts": self.gusts, "rtk": self.rtk,
                "rain": self.rain, "rain_rule": self.rain_rule}


def _number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_scenario(data: Any) -> list[str]:
    """Every problem with a scenario dict, as readable messages ([] = valid)."""
    if not isinstance(data, dict):
        return ["a scenario must be a JSON object"]
    errors: list[str] = []
    if not isinstance(data.get("name"), str) or not data["name"].strip():
        errors.append("name: give the scenario a name")
    if "seed" in data and not (isinstance(data["seed"], int) and not isinstance(data["seed"], bool)
                               and 0 <= data["seed"] < 2 ** 31):
        errors.append("seed: a whole number from 0 to 2147483647")

    def check_keys(channel: str, fields: dict[str, tuple[float, float] | None], extra=None) -> None:
        keys = data.get(channel, [])
        if not isinstance(keys, list):
            errors.append(f"{channel}: a list of keys")
            return
        for i, key in enumerate(keys):
            where = f"{channel}[{i}]"
            if not isinstance(key, dict):
                errors.append(f"{where}: a key must be an object")
                continue
            if not _number(key.get("t")) or key["t"] < 0:
                errors.append(f"{where}.t: show time in seconds, 0 or more")
            for name, bounds in fields.items():
                value = key.get(name)
                if not _number(value):
                    errors.append(f"{where}.{name}: a number")
                elif bounds and not bounds[0] <= value <= bounds[1]:
                    errors.append(f"{where}.{name}: between {bounds[0]:g} and {bounds[1]:g}")
            if extra:
                extra(where, key)

    def check_rtk(where: str, key: dict) -> None:
        if key.get("state") not in RTK_STATES:
            errors.append(f"{where}.state: one of {', '.join(RTK_STATES)}")

    check_keys("wind", {"speed_mps": LIMITS["speed_mps"], "from_deg": None, "turbulence": LIMITS["turbulence"]})
    check_keys("gusts", {"peak_mps": LIMITS["peak_mps"], "duration_s": LIMITS["duration_s"], "from_deg": None})
    check_keys("rtk", {}, check_rtk)
    check_keys("rain", {"mm_h": LIMITS["mm_h"]})

    rule = data.get("rain_rule")
    if rule is not None:
        if not isinstance(rule, dict):
            errors.append("rain_rule: an object")
        else:
            for name in DEFAULT_RAIN_RULE:
                if name in rule and (not _number(rule[name]) or rule[name] < 0):
                    errors.append(f"rain_rule.{name}: a number, 0 or more")
            alert = rule.get("alert_mm_h", DEFAULT_RAIN_RULE["alert_mm_h"])
            limit = rule.get("limit_mm_h", DEFAULT_RAIN_RULE["limit_mm_h"])
            if _number(alert) and _number(limit) and limit <= alert:
                errors.append("rain_rule: the limit level must be above the alert level")
    return errors


def folder_name(name: str) -> str:
    """The scenario's folder under stage3/scenarios/ (ASCII, filesystem-safe)."""
    slug = re.sub(r"[^A-Za-z0-9_-]+", "-", name.strip()).strip("-").lower()
    return slug[:60] or "scenario"


# --------------------------------------------------------------------------- #
# Interpolation
# --------------------------------------------------------------------------- #

def _linear(keys: list[dict], name: str, t: np.ndarray, default: float) -> np.ndarray:
    if not keys:
        return np.full(t.shape, default, float)
    return np.interp(t, [k["t"] for k in keys], [k[name] for k in keys])


def _unwrapped_directions(keys: list[dict]) -> list[float]:
    """Key directions made continuous so that linear interpolation turns the short way round."""
    out: list[float] = []
    for key in keys:
        d = float(key["from_deg"])
        if out:
            d = out[-1] + ((d - out[-1] + 180.0) % 360.0 - 180.0)
        out.append(d)
    return out


def wind_at(scenario: Scenario, t) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(speed m/s, from_deg in [0, 360), turbulence intensity) at show time(s) t."""
    t = np.asarray(t, float)
    keys = scenario.wind
    speed = _linear(keys, "speed_mps", t, 0.0)
    turb = _linear(keys, "turbulence", t, 0.0)
    if keys:
        direction = np.interp(t, [k["t"] for k in keys], _unwrapped_directions(keys)) % 360.0
    else:
        direction = np.zeros(t.shape)
    return speed, direction, turb


def rain_at(scenario: Scenario, t) -> np.ndarray:
    return _linear(scenario.rain, "mm_h", np.asarray(t, float), 0.0)


def rtk_index_at(scenario: Scenario, t) -> np.ndarray:
    """Index into RTK_STATES at show time(s) t: the last key at or before t (the first key before it)."""
    t = np.asarray(t, float)
    if not scenario.rtk:
        return np.zeros(t.shape, int)
    times = np.array([k["t"] for k in scenario.rtk])
    states = np.array([RTK_STATES.index(k["state"]) for k in scenario.rtk])
    return states[np.clip(np.searchsorted(times, t, side="right") - 1, 0, len(times) - 1)]


def rtk_state_at(scenario: Scenario, t: float) -> str:
    return RTK_STATES[int(rtk_index_at(scenario, t))]


def toward_unit(from_deg) -> np.ndarray:
    """ENU unit vector(s) of the direction the air moves *to*, for wind blowing from `from_deg`."""
    rad = np.radians(np.asarray(from_deg, float))
    return np.stack([-np.sin(rad), -np.cos(rad), np.zeros(rad.shape)], axis=-1)


def rain_label(mm_h: float) -> str:
    if mm_h <= 0.0:
        return "none"
    return next(label for label, upper in RAIN_SCALE if mm_h < upper)


# --------------------------------------------------------------------------- #
# Kernel inputs
# --------------------------------------------------------------------------- #

@dataclass
class WeatherTable:
    """Rows of WX_STRIDE values at `hz`, row r at show time r / hz (the last row holds)."""

    hz: float
    rows: np.ndarray            # (R, WX_STRIDE) float64

    @property
    def times(self) -> np.ndarray:
        return np.arange(self.rows.shape[0]) / self.hz


def rtk_noise(profile: DroneProfile, index: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    noise = np.array([profile.get("tolerances.gnss_rtk_noise_m"), *(v[0] for v in RTK_DEGRADED.values())])
    drift = np.array([profile.get("tolerances.gnss_drift_rate_m_per_sqrt_s"),
                      *(v[1] for v in RTK_DEGRADED.values())])
    return noise[index], drift[index]


def tabulate(scenario: Scenario, duration_sec: float, profile: DroneProfile, hz: float = TABLE_HZ) -> WeatherTable:
    """The timeline sampled at `hz` from t = 0 to at least `duration_sec`."""
    count = int(math.ceil(max(duration_sec, 0.0) * hz)) + 2
    t = np.arange(count) / hz
    speed, direction, turb = wind_at(scenario, t)
    rows = np.zeros((count, WX_STRIDE))
    rows[:, WX_WIND:WX_WIND + 3] = toward_unit(direction) * speed[:, None]
    rows[:, WX_TURB] = np.maximum(TURBULENCE_FLOOR_MPS, turb * speed)
    # Frozen turbulence is carried by the mean wind (never slower than 1 m/s, as in the stress test).
    carry = np.maximum(speed, 1.0)
    rows[1:, WX_ADVECT] = np.cumsum(0.5 * (carry[1:] + carry[:-1]) / hz)
    rows[:, WX_GNSS_NOISE], rows[:, WX_GNSS_DRIFT] = rtk_noise(profile, rtk_index_at(scenario, t))
    rows[:, WX_RAIN] = rain_at(scenario, t)
    return WeatherTable(hz=hz, rows=rows)


def gust_fronts(scenario: Scenario, field_center_xy) -> np.ndarray:
    """(G, 9) gust rows for the kernels: peak vector xyz, sweep direction xyz, start, duration, speed.

    A gust event at time t reaches the field centre at t; it sweeps at the wind speed of that moment."""
    cx, cy = float(field_center_xy[0]), float(field_center_xy[1])
    out = np.zeros((len(scenario.gusts), 9))
    for row, gust in zip(out, scenario.gusts):
        unit = toward_unit(gust["from_deg"])
        wind_speed = float(wind_at(scenario, gust["t"])[0])
        speed = max(wind_speed, GUST_MIN_SPEED_MPS)
        row[0:3] = unit * gust["peak_mps"]
        row[3:6] = unit
        row[6] = gust["t"] - (unit[0] * cx + unit[1] * cy) / speed
        row[7] = gust["duration_s"]
        row[8] = speed
    return out


def scenario_disturbances(scenario: Scenario, profile: DroneProfile, fleet_size: int, duration_sec: float,
                          field_center_xy=(0.0, 0.0)):
    """The `Disturbances` of one flight under `scenario`.

    Turbulence modes, launch placement, GNSS noise and per-drone hardware spread are drawn from the
    scenario's seed with the Monte Carlo stress test's distributions; temperature is the Stage 3 default."""
    from .simulator import Disturbances

    rng = np.random.default_rng(scenario.seed)
    tol = lambda key: profile.get(f"tolerances.{key}") / 100.0  # noqa: E731

    def spread(nominal: float, pct: float) -> np.ndarray:
        return nominal * (1.0 + rng.uniform(-pct, pct, fleet_size))

    # Unit-sigma modes: the table's WX_TURB scales them, WX_ADVECT moves them.
    m = TURBULENCE_MODES
    wavelength = rng.uniform(20.0, 200.0, m)
    k_heading = rng.uniform(0.0, 2.0 * math.pi, m)
    k_mag = 2.0 * math.pi / wavelength
    mode_k = np.stack([k_mag * np.cos(k_heading), k_mag * np.sin(k_heading), np.zeros(m)], axis=1)
    amp_dir = rng.normal(size=(m, 3)) * np.array([1.0, 1.0, 0.5])
    amp_dir /= np.linalg.norm(amp_dir, axis=1, keepdims=True)
    mode_amp = amp_dir * math.sqrt(2.0 / m)
    mode_omega = k_mag * rng.uniform(0.5, 1.5, m)        # rad per metre carried
    mode_phase = rng.uniform(0.0, 2.0 * math.pi, m)

    table = tabulate(scenario, duration_sec, profile)
    return Disturbances(
        seed=scenario.seed,
        ambient_c=AMBIENT_C,
        mean_wind=table.rows[0, WX_WIND:WX_WIND + 3].copy(),
        mode_amp=mode_amp,
        mode_k=mode_k,
        mode_omega=mode_omega,
        mode_phase=mode_phase,
        gust_vec=np.zeros(3),
        gust_dir=np.array([1.0, 0.0, 0.0]),
        gust_start=0.0,
        gust_duration=0.0,
        gust_speed=1.0,
        mass_kg=spread(profile.get("physical.mass_kg"), tol("mass_pct")),
        max_thrust_n=spread(profile.get("motor_prop.max_thrust_per_motor_n"), tol("max_thrust_pct")),
        motor_tau_s=spread(profile.get("motor_prop.motor_time_constant_ms") / 1000.0, tol("motor_time_constant_pct")),
        drag_cd=spread(profile.get("physical.drag_coefficient_cd"), tol("drag_coefficient_pct")),
        capacity_ah=spread(profile.get("battery.capacity_mah") / 1000.0, tol("battery_capacity_pct")),
        internal_resistance_ohm=spread(profile.get("battery.internal_resistance_ohm"), tol("internal_resistance_pct")),
        initial_soc=profile.get("battery.initial_soc")
        * rng.uniform(profile.get("tolerances.initial_soc_min"), 1.0, fleet_size),
        gnss_noise_m=float(table.rows[0, WX_GNSS_NOISE]),
        gnss_drift_m_per_sqrt_s=float(table.rows[0, WX_GNSS_DRIFT]),
        initial_position_error_m=profile.get("tolerances.initial_position_error_m"),
        gusts=gust_fronts(scenario, field_center_xy),
        weather=table,
    )
