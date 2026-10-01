// Heads-up display of a simulated flight's weather (docs/4-condition_simulator.md §6): wind speed and
// direction, gusts passing, rain against the alert and limit levels, RTK state.

import type { SimWeather } from "./types";
import { compass, rainLabel, RTK_LABEL, weatherAt } from "./weather";

export function WeatherHud({ weather, time }: { weather: SimWeather; time: number }) {
  const now = weatherAt(weather, time);
  const { alert_mm_h: alert, limit_mm_h: limit } = weather.rain_rule;
  // The rain gauge runs to 1.5 x the limit level so both marks sit inside it.
  const top = Math.max(limit * 1.5, 1);
  const pct = (v: number) => `${Math.min(100, (v / top) * 100)}%`;
  const rainTone = now.rain >= limit ? "bad" : now.rain >= alert ? "warn" : "ok";
  const gust = now.gusts.reduce((best, g) => (g.peak * g.strength > best ? g.peak * g.strength : best), 0);

  return (
    <div className="hud" aria-label="Weather now">
      <div className="hud-row">
        <svg className="hud-arrow" viewBox="-12 -12 24 24" aria-hidden="true">
          <circle r="11" className="hud-dial" />
          {/* Points where the air goes: from the wind direction, turned half a circle. */}
          <g transform={`rotate(${now.fromDeg + 180})`} style={{ opacity: now.speed < 0.3 ? 0.25 : 1 }}>
            <path d="M0 -8 L4 2 L0 0 L-4 2 Z" />
          </g>
        </svg>
        <div>
          <p className="hud-value">
            <span className="num">{now.speed.toFixed(1)}</span> m/s
          </p>
          <p className="hud-label">
            {now.speed < 0.3 ? "Calm" : `from ${compass(now.fromDeg)} (${Math.round(now.fromDeg)}°)`}
            {now.turbulence > 0 && now.speed >= 0.3 && <>, turbulence {Math.round(now.turbulence * 100)} %</>}
          </p>
        </div>
      </div>
      {gust > 0.05 && (
        <p className="hud-gust">
          Gust <span className="num">+{gust.toFixed(1)}</span> m/s
        </p>
      )}
      <div className="hud-rain">
        <p className="hud-label">
          {rainLabel(now.rain)} <span className={`num tone-${rainTone}`}>{now.rain.toFixed(1)}</span> mm/h
        </p>
        <div className="hud-gauge" role="img" aria-label={`Rain ${now.rain.toFixed(1)} mm/h, alert at ${alert}, limit at ${limit}`}>
          <span className={`hud-gauge-fill hud-gauge-${rainTone}`} style={{ width: pct(now.rain) }} />
          <span className="hud-tick hud-tick-alert" style={{ left: pct(alert) }} title={`Alert level ${alert} mm/h`} />
          <span className="hud-tick hud-tick-limit" style={{ left: pct(limit) }} title={`Limit level ${limit} mm/h`} />
        </div>
      </div>
      <p className={`hud-rtk hud-rtk-${now.rtk}`}>{RTK_LABEL[now.rtk] ?? now.rtk}</p>
    </div>
  );
}
