// Two runs' closest-pair distance over show time, overlaid (docs/5-studio_gui.md §6.5).
// No Tauri imports: it takes the two separation series like the rest of src/replay.

import { useEffect, useMemo, useRef, useState } from "react";
import { formatTime, metres, nearestFrame } from "./sampling";
import type { Separation } from "./types";

export interface CompareSeries {
  label: string;
  separation: Separation;
  floor: number | null; // required distance this run was checked against
}

interface Props {
  a: CompareSeries;
  b: CompareSeries;
}

// Series colours stay out of the status palette: this run in the primary orange, the other in the cyan accent.
export const SERIES_COLORS = ["#ffa34d", "#00d9ff"] as const;
const COLORS = { bg: "#0f0f0f", grid: "#222222", axis: "#a0a0a0", floor: "#facc15", bad: "#ef4444" };
const PAD = { left: 56, right: 16, top: 14, bottom: 22 };

function niceStep(span: number, target: number): number {
  const raw = span / target;
  const mag = 10 ** Math.floor(Math.log10(raw));
  return [1, 2, 5, 10].map((m) => m * mag).find((s) => s >= raw) ?? 10 * mag;
}

const span = (s: Separation) => [s.times[0] ?? 0, s.times[s.times.length - 1] ?? 0] as const;

export function SeparationCompare({ a, b }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ w: 0, h: 0 });
  const [hover, setHover] = useState<number | null>(null);

  const [t0, t1] = useMemo(() => {
    const [a0, a1] = span(a.separation);
    const [b0, b1] = span(b.separation);
    return [Math.min(a0, b0), Math.max(a1, b1, Math.min(a0, b0) + 1e-6)];
  }, [a.separation, b.separation]);
  const yMax = Math.max(a.floor ?? 0, b.floor ?? 0, 1) * 2.5; // the interesting band is near the floor

  useEffect(() => {
    const canvas = canvasRef.current!;
    const ro = new ResizeObserver(([entry]) => setSize({ w: entry.contentRect.width, h: entry.contentRect.height }));
    ro.observe(canvas);
    return () => ro.disconnect();
  }, []);

  const xOf = (t: number) => PAD.left + ((t - t0) / (t1 - t0)) * (size.w - PAD.left - PAD.right);
  const tOf = (x: number) =>
    Math.min(t1, Math.max(t0, t0 + ((x - PAD.left) / Math.max(size.w - PAD.left - PAD.right, 1)) * (t1 - t0)));
  const yOf = (d: number) => PAD.top + (1 - Math.min(d, yMax) / yMax) * (size.h - PAD.top - PAD.bottom);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !size.w) return;
    const dpr = window.devicePixelRatio || 1;
    canvas.width = Math.round(size.w * dpr);
    canvas.height = Math.round(size.h * dpr);
    const ctx = canvas.getContext("2d")!;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = COLORS.bg;
    ctx.fillRect(0, 0, size.w, size.h);
    ctx.font = "12px Inter, system-ui, 'Segoe UI', sans-serif";

    const dStep = niceStep(yMax, 4);
    ctx.fillStyle = COLORS.axis;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let d = 0; d <= yMax + 1e-9; d += dStep) {
      const y = yOf(d);
      ctx.strokeStyle = COLORS.grid;
      ctx.beginPath();
      ctx.moveTo(PAD.left, y);
      ctx.lineTo(size.w - PAD.right, y);
      ctx.stroke();
      ctx.fillText(`${d.toFixed(dStep < 1 ? 1 : 0)} m`, PAD.left - 8, y);
    }
    const tStep = niceStep(t1 - t0, Math.max(2, Math.floor(size.w / 110)));
    ctx.textAlign = "center";
    ctx.textBaseline = "alphabetic";
    for (let t = Math.ceil(t0 / tStep) * tStep; t <= t1 + 1e-9; t += tStep) {
      ctx.fillText(formatTime(t).replace(/\.0$/, ""), xOf(t), size.h - 6);
    }

    // Required distances: one line when both runs used the same, else one per run in its colour.
    const floors =
      a.floor !== null && a.floor === b.floor
        ? [{ d: a.floor, color: COLORS.floor, label: `${a.floor.toFixed(2)} m required` }]
        : [a, b].flatMap((s, i) =>
            s.floor === null ? [] : [{ d: s.floor, color: SERIES_COLORS[i], label: `${s.floor.toFixed(2)} m (${s.label})` }],
          );
    ctx.setLineDash([5, 4]);
    for (const f of floors) {
      const y = yOf(f.d);
      ctx.strokeStyle = f.color;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(PAD.left, y);
      ctx.lineTo(size.w - PAD.right, y);
      ctx.stroke();
    }
    ctx.setLineDash([]);
    ctx.textAlign = "right";
    ctx.textBaseline = "bottom";
    floors.forEach((f, i) => {
      ctx.fillStyle = f.color;
      ctx.fillText(f.label, size.w - PAD.right - 4, yOf(f.d) - 3 - i * 14);
    });

    // The other run first, so this run's line sits on top.
    [b, a].forEach((s, k) => {
      const color = SERIES_COLORS[k === 0 ? 1 : 0];
      const { times, min_m } = s.separation;
      ctx.strokeStyle = color;
      ctx.lineJoin = "round";
      for (let i = 1; i < times.length; i++) {
        const p = min_m[i - 1];
        const q = min_m[i];
        if (p === null || q === null) continue;
        ctx.lineWidth = s.floor !== null && (p < s.floor || q < s.floor) ? 2.75 : 1.5;
        ctx.beginPath();
        ctx.moveTo(xOf(times[i - 1]), yOf(p));
        ctx.lineTo(xOf(times[i]), yOf(q));
        ctx.stroke();
      }
      const w = s.separation.worst;
      if (w.distance_m !== null) {
        const below = s.floor !== null && w.distance_m < s.floor;
        ctx.fillStyle = below ? COLORS.bad : color;
        ctx.beginPath();
        ctx.arc(xOf(w.time_sec), yOf(w.distance_m), 4, 0, Math.PI * 2);
        ctx.fill();
      }
    });

    if (hover !== null) {
      ctx.strokeStyle = COLORS.axis;
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(xOf(hover), PAD.top - 4);
      ctx.lineTo(xOf(hover), size.h - PAD.bottom);
      ctx.stroke();
    }
  }, [size, a, b, t0, t1, yMax, hover]);

  const readout = (s: CompareSeries) => {
    const t = hover ?? s.separation.worst.time_sec;
    const [s0, s1] = span(s.separation);
    if (t < s0 - 1e-6 || t > s1 + 1e-6) return { value: null, pair: null };
    const k = nearestFrame(s.separation.times, t);
    return { value: s.separation.min_m[k], pair: [s.separation.a[k], s.separation.b[k]] as const };
  };

  return (
    <div className="compare-chart">
      <div className="compare-readout" aria-live="off">
        <span className="strip-time">{hover === null ? "Closest moment of each" : formatTime(hover)}</span>
        {[a, b].map((s, i) => {
          const r = readout(s);
          const below = s.floor !== null && r.value !== null && r.value < s.floor;
          return (
            <span key={i} className="compare-value">
              <span className="series-swatch" style={{ background: SERIES_COLORS[i] }} aria-hidden="true" />
              {s.label}: <strong className={below ? "tone-bad" : undefined}>{metres(r.value, 2)}</strong>
              {r.pair && r.value !== null && (
                <span className="muted">
                  {" "}
                  drones {r.pair[0]} and {r.pair[1]}
                  {hover === null && <> at {formatTime(s.separation.worst.time_sec)}</>}
                </span>
              )}
            </span>
          );
        })}
      </div>
      <canvas
        ref={canvasRef}
        className="compare-canvas"
        role="img"
        aria-label={`Closest pair distance over time. ${a.label}: closest ${metres(a.separation.worst.distance_m, 2)}. ${b.label}: closest ${metres(b.separation.worst.distance_m, 2)}.`}
        onPointerMove={(e) => {
          const rect = e.currentTarget.getBoundingClientRect();
          setHover(tOf(e.clientX - rect.left));
        }}
        onPointerLeave={() => setHover(null)}
      />
    </div>
  );
}
