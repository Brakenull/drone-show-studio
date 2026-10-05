// Weather of a simulated flight drawn in the replay scene:
// wind as streaks drifting with the air, denser with more turbulence; rain as falling streaks,
// denser with intensity; each gust front as a translucent band sweeping across the field.
// Everything is a function of playback time, so scrubbing shows the same picture as playing.
// ENU (x east, y north, z up) maps to three.js as (x, z, -y).

import * as THREE from "three";
import { carryAt, carryTable, gustStrength, towardUnit, weatherAt } from "./weather";
import type { ReplayHeader, SimWeather } from "./types";

export const WIND_COLOR = 0x7dd3c0;
export const RAIN_COLOR = 0x6b9cff;
const GUST_COLOR = 0xe6e6e6;

const MAX_WIND = 360;
const MAX_RAIN = 1800;
const RAIN_FALL_MPS = 8;
const HEAVY_RAIN_MM_H = 8; // rain streaks reach full density here

/** Deterministic pseudo-random numbers, so the streaks don't jump between renders. */
function rng(seed: number) {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

const wrap = (v: number, lo: number, size: number) => lo + ((((v - lo) % size) + size) % size);

export class WeatherLayer {
  readonly group = new THREE.Group();
  private wind: THREE.LineSegments;
  private rain: THREE.LineSegments;
  private gusts: THREE.Mesh[] = [];
  private windSeeds: Float32Array; // x, y, z, phase per streak
  private rainSeeds: Float32Array; // x, y, z per drop
  private carry: Float32Array;
  private box: { x0: number; y0: number; z0: number; w: number; d: number; h: number };

  constructor(
    private weather: SimWeather,
    header: ReplayHeader,
  ) {
    const { bounds_min: lo, bounds_max: hi, ground_z_m } = header;
    const pad = 25;
    this.box = {
      x0: lo[0] - pad,
      y0: lo[1] - pad,
      z0: ground_z_m + 1,
      w: hi[0] - lo[0] + 2 * pad,
      d: hi[1] - lo[1] + 2 * pad,
      h: Math.max(hi[2] - ground_z_m + 15, 20),
    };
    this.carry = carryTable(weather);

    const r = rng(20260924);
    this.windSeeds = new Float32Array(MAX_WIND * 4).map((_, i) => r() * (i % 4 === 3 ? Math.PI * 2 : 1));
    this.rainSeeds = new Float32Array(MAX_RAIN * 3).map(() => r());

    this.wind = this.streaks(MAX_WIND, WIND_COLOR, 0.38);
    this.rain = this.streaks(MAX_RAIN, RAIN_COLOR, 0.42);
    this.group.add(this.wind, this.rain);

    const length = Math.max(this.box.w, this.box.d);
    for (const g of weather.gusts) {
      const band = new THREE.Mesh(
        new THREE.BoxGeometry(Math.max(g.duration_s * g.speed_mps, 1), this.box.h, length),
        new THREE.MeshBasicMaterial({
          color: GUST_COLOR,
          transparent: true,
          opacity: 0,
          depthWrite: false,
          blending: THREE.AdditiveBlending,
        }),
      );
      // Long side across the front: rotate the box's x axis onto the gust direction.
      band.rotation.y = Math.atan2(g.dir[1], g.dir[0]);
      band.visible = false;
      this.gusts.push(band);
      this.group.add(band);
    }
  }

  private streaks(count: number, color: number, opacity: number): THREE.LineSegments {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(count * 6), 3));
    geometry.setDrawRange(0, 0);
    const lines = new THREE.LineSegments(
      geometry,
      new THREE.LineBasicMaterial({ color, transparent: true, opacity, depthWrite: false }),
    );
    lines.frustumCulled = false;
    return lines;
  }

  update(t: number) {
    const now = weatherAt(this.weather, t);
    const { x0, y0, z0, w, d, h } = this.box;
    const [ux, uy] = towardUnit(now.fromDeg);
    const [carryX, carryY] = carryAt(this.carry, this.weather.hz, t);

    // ---- wind: streaks moving with the air; more of them, and more wobble, in turbulent air ----
    const calm = now.speed < 0.3;
    const windCount = calm ? 0 : Math.round(MAX_WIND * Math.min(1, 0.25 + now.turbulence * 2.5));
    const windLen = Math.min(0.45 * now.speed + 0.6, 6);
    const wp = (this.wind.geometry.getAttribute("position") as THREE.BufferAttribute).array as Float32Array;
    for (let i = 0; i < windCount; i++) {
      const s = this.windSeeds.subarray(i * 4, i * 4 + 4);
      const x = wrap(x0 + s[0] * w + carryX, x0, w);
      const y = wrap(y0 + s[1] * d + carryY, y0, d);
      const z = z0 + s[2] * h + now.turbulence * 3 * Math.sin(s[3] + t * 1.7);
      wp.set([x, z, -y, x + ux * windLen, z, -(y + uy * windLen)], i * 6);
    }
    this.wind.geometry.setDrawRange(0, windCount * 2);
    this.wind.geometry.getAttribute("position").needsUpdate = true;

    // ---- rain: falling streaks, slanted by the wind ----
    const rainCount = now.rain > 0 ? Math.round(MAX_RAIN * Math.min(1, 0.04 + now.rain / HEAVY_RAIN_MM_H)) : 0;
    const rp = (this.rain.geometry.getAttribute("position") as THREE.BufferAttribute).array as Float32Array;
    const slant = 1.2 / RAIN_FALL_MPS;
    for (let i = 0; i < rainCount; i++) {
      const s = this.rainSeeds.subarray(i * 3, i * 3 + 3);
      const x = wrap(x0 + s[0] * w + carryX, x0, w);
      const y = wrap(y0 + s[1] * d + carryY, y0, d);
      const z = z0 + h - wrap(s[2] * h + RAIN_FALL_MPS * t, 0, h);
      const dx = ux * now.speed * slant;
      const dy = uy * now.speed * slant;
      rp.set([x, z, -y, x - dx, z + 1.2, -(y - dy)], i * 6);
    }
    this.rain.geometry.setDrawRange(0, rainCount * 2);
    this.rain.geometry.getAttribute("position").needsUpdate = true;

    // ---- gusts: the band between the front's leading and trailing edge ----
    const [cx, cy] = this.weather.field_center;
    this.weather.gusts.forEach((g, i) => {
      const band = this.gusts[i];
      const lead = (t - g.start_sec) * g.speed_mps; // distance along dir of the leading edge
      const tail = lead - g.duration_s * g.speed_mps;
      const centreAlong = g.dir[0] * cx + g.dir[1] * cy;
      const reach = Math.hypot(w, d) / 2;
      const visible = lead > centreAlong - reach && tail < centreAlong + reach;
      band.visible = visible;
      if (!visible) return;
      const mid = (lead + tail) / 2;
      // Centre of the band: on the line through the field centre along dir, at distance `mid`.
      const px = cx + g.dir[0] * (mid - centreAlong);
      const py = cy + g.dir[1] * (mid - centreAlong);
      band.position.set(px, z0 + h / 2, -py);
      // Fades in as the front reaches the field centre and out as it leaves.
      const atCentre = gustStrength(g, centreAlong, t);
      (band.material as THREE.MeshBasicMaterial).opacity = (0.012 + 0.035 * atCentre) * Math.min(1, 0.4 + g.peak_mps / 8);
    });
  }

  dispose() {
    this.group.traverse((obj) => {
      const mesh = obj as THREE.Mesh;
      mesh.geometry?.dispose();
      (mesh.material as THREE.Material | undefined)?.dispose();
    });
  }
}
