// The scenario weather of a simulated flight at any playback time.
// Samples come from the bridge at `hz`; wind and rain are blended, RTK steps.

import type { SimWeather } from "./types";

export interface WeatherNow {
  speed: number; // m/s
  fromDeg: number; // direction the wind blows from, 0 = north
  turbulence: number; // sigma / mean speed
  rain: number; // mm/h
  rtk: string;
  /** Gusts passing the field centre now, with their strength there (0..1 of the peak). */
  gusts: { peak: number; strength: number; fromDeg: number }[];
}

function blend(values: number[], hz: number, t: number): number {
  if (!values.length) return 0;
  const x = Math.max(0, t * hz);
  const i = Math.min(Math.floor(x), values.length - 1);
  const j = Math.min(i + 1, values.length - 1);
  return values[i] + (values[j] - values[i]) * Math.min(1, x - i);
}

/** Direction blended the short way round. */
function blendAngle(values: number[], hz: number, t: number): number {
  if (!values.length) return 0;
  const x = Math.max(0, t * hz);
  const i = Math.min(Math.floor(x), values.length - 1);
  const j = Math.min(i + 1, values.length - 1);
  const d = ((values[j] - values[i] + 540) % 360) - 180;
  return (values[i] + d * Math.min(1, x - i) + 360) % 360;
}

/** Fraction (0..1) of a gust's peak at a point `along` metres down its direction, at time t. */
export function gustStrength(g: SimWeather["gusts"][number], along: number, t: number): number {
  const local = t - g.start_sec - along / Math.max(g.speed_mps, 0.1);
  if (local <= 0 || local >= g.duration_s) return 0;
  return 0.5 * (1 - Math.cos((2 * Math.PI * local) / g.duration_s));
}

export function weatherAt(w: SimWeather, t: number): WeatherNow {
  const k = Math.min(Math.max(Math.floor(t * w.hz), 0), Math.max(w.rtk.length - 1, 0));
  const [cx, cy] = w.field_center;
  return {
    speed: blend(w.speed_mps, w.hz, t),
    fromDeg: blendAngle(w.from_deg, w.hz, t),
    turbulence: blend(w.turbulence, w.hz, t),
    rain: blend(w.rain_mm_h, w.hz, t),
    rtk: w.rtk_states[w.rtk[k] ?? 0] ?? "fixed",
    gusts: w.gusts
      .map((g) => ({ peak: g.peak_mps, fromDeg: g.from_deg, strength: gustStrength(g, g.dir[0] * cx + g.dir[1] * cy, t) }))
      .filter((g) => g.strength > 0.02),
  };
}

/** ENU (x east, y north) unit vector the air moves toward, for wind from `fromDeg`. */
export function towardUnit(fromDeg: number): [number, number] {
  const r = (fromDeg * Math.PI) / 180;
  return [-Math.sin(r), -Math.cos(r)];
}

/** How far the air has carried things since t = 0 (ENU metres), sampled at `hz`: for drifting streaks. */
export function carryTable(w: SimWeather): Float32Array {
  const n = w.speed_mps.length;
  const out = new Float32Array(n * 2);
  for (let i = 1; i < n; i++) {
    const [ax, ay] = towardUnit(w.from_deg[i - 1]);
    const [bx, by] = towardUnit(w.from_deg[i]);
    const dt = 1 / w.hz;
    out[i * 2] = out[(i - 1) * 2] + 0.5 * (ax * w.speed_mps[i - 1] + bx * w.speed_mps[i]) * dt;
    out[i * 2 + 1] = out[(i - 1) * 2 + 1] + 0.5 * (ay * w.speed_mps[i - 1] + by * w.speed_mps[i]) * dt;
  }
  return out;
}

export function carryAt(table: Float32Array, hz: number, t: number): [number, number] {
  const n = table.length / 2;
  if (!n) return [0, 0];
  const x = Math.max(0, t * hz);
  const i = Math.min(Math.floor(x), n - 1);
  const j = Math.min(i + 1, n - 1);
  const f = Math.min(1, x - i);
  return [table[i * 2] + (table[j * 2] - table[i * 2]) * f, table[i * 2 + 1] + (table[j * 2 + 1] - table[i * 2 + 1]) * f];
}

const COMPASS = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"];
export const compass = (deg: number) => COMPASS[Math.round((((deg % 360) + 360) % 360) / 22.5) % 16];

/** Meteorological rain scale. */
export function rainLabel(mmH: number): string {
  if (mmH <= 0) return "No rain";
  if (mmH < 0.5) return "Drizzle";
  if (mmH < 2.5) return "Light rain";
  if (mmH < 7.6) return "Moderate rain";
  return "Heavy rain";
}

export const RTK_LABEL: Record<string, string> = {
  fixed: "RTK fixed",
  float: "RTK float",
  gps: "GPS only",
};
