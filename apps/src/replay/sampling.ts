// Frame lookup and interpolation over the replay arrays.

import type { ReplayData, V3 } from "./types";

/** Frame k and blend fraction for time t (times are uniform except possibly the last step). */
export function locate(times: number[], t: number): { k: number; frac: number } {
  const last = times.length - 1;
  if (t <= times[0]) return { k: 0, frac: 0 };
  if (t >= times[last]) return { k: last, frac: 0 };
  let lo = 0;
  let hi = last;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (times[mid] <= t) lo = mid;
    else hi = mid;
  }
  return { k: lo, frac: (t - times[lo]) / (times[hi] - times[lo]) };
}

/** Nearest sampled frame, for per-frame readouts. */
export function nearestFrame(times: number[], t: number): number {
  const { k, frac } = locate(times, t);
  return frac > 0.5 && k + 1 < times.length ? k + 1 : k;
}

function sampleFrames(data: ReplayData, frames: Float32Array, drone: number, t: number, out: V3): V3 {
  const { k, frac } = locate(data.separation.times, t);
  const n = data.header.fleet_size;
  const i0 = (k * n + drone) * 3;
  const k1 = Math.min(k + 1, data.header.frames - 1);
  const i1 = (k1 * n + drone) * 3;
  const p = frames;
  out[0] = p[i0] + (p[i1] - p[i0]) * frac;
  out[1] = p[i0 + 1] + (p[i1 + 1] - p[i0 + 1]) * frac;
  out[2] = p[i0 + 2] + (p[i1 + 2] - p[i0 + 2]) * frac;
  return out;
}

export function positionAt(data: ReplayData, drone: number, t: number, out: V3 = [0, 0, 0]): V3 {
  return sampleFrames(data, data.positions, drone, t, out);
}

/** Planned position of a drone in a simulated flight (the flown one when there is no plan). */
export function referenceAt(data: ReplayData, drone: number, t: number, out: V3 = [0, 0, 0]): V3 {
  return sampleFrames(data, data.reference ?? data.positions, drone, t, out);
}

export function distanceAt(data: ReplayData, a: number, b: number, t: number): number {
  const pa = positionAt(data, a, t);
  const pb = positionAt(data, b, t);
  return Math.hypot(pa[0] - pb[0], pa[1] - pb[1], pa[2] - pb[2]);
}

/** Closest other drone to `drone` at time t (O(n); used for the picked-drone readout). */
export function nearestTo(data: ReplayData, drone: number, t: number): { other: number; distance: number } {
  const n = data.header.fleet_size;
  const p = positionAt(data, drone, t);
  const q: V3 = [0, 0, 0];
  let best = { other: -1, distance: Infinity };
  for (let j = 0; j < n; j++) {
    if (j === drone) continue;
    positionAt(data, j, t, q);
    const d = Math.hypot(p[0] - q[0], p[1] - q[1], p[2] - q[2]);
    if (d < best.distance) best = { other: j, distance: d };
  }
  return best;
}

export function formatTime(t: number): string {
  const sign = t < 0 ? "-" : "";
  const s = Math.abs(t);
  const m = Math.floor(s / 60);
  const rest = s - m * 60;
  return `${sign}${m}:${rest.toFixed(1).padStart(4, "0")}`;
}

export const metres = (d: number | null | undefined, digits = 3) =>
  d === null || d === undefined || !Number.isFinite(d) ? "n/a" : `${d.toFixed(digits)} m`;
