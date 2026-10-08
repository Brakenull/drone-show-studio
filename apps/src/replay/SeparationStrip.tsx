// Closest-pair distance over time, drawn as the replay's scrubber.

import { useEffect, useMemo, useRef, useState } from "react";
import { formatTime, metres, nearestFrame } from "./sampling";
import { tokens } from "../theme";
import { formationDetail } from "./formations";
import type { FormationMark, Separation, ShowTimeline } from "./types";

interface Props {
  separation: Separation;
  time: number;
  t0: number;
  t1: number;
  floor: number | null; // gatekeeper floor: below it Stage 2 rejects a show
  /** Text after the floor value on its line (default "required"). */
  floorLabel?: string;
  nominal: number | null; // min_distance_m from the Phase 1 file
  /** Shaded window: a rejected transition ("bad"), or a return path's flight home ("info"). */
  span: { start: number; end: number; label: string; tone?: "bad" | "info" } | null;
  /** Simulated flights: per frame, the largest gap between a drone and its planned position (second line). */
  deviation?: number[] | null;
  /** Moments marked across the strip, e.g. when the rain reaches its alert and limit levels. */
  marks?: { time: number; label: string; tone: "warn" | "bad" }[];
  /** Each formation at the time the planned show reaches it, and the takeoff and return legs. */
  timeline?: ShowTimeline | null;
  onSeek: (t: number) => void;
}

const COLORS = {
  bg: tokens.colorBgLayout,
  grid: tokens.colorBorderSecondary,
  axis: tokens.colorTextSecondary,
  line: tokens.colorSuccess,
  bad: tokens.colorError,
  floor: tokens.colorWarning,
  // Hex colour plus a two-digit alpha.
  span: `${tokens.colorError}12`,
  spanInfo: `${tokens.colorInfo}14`,
  playhead: "#ffa34d", // replay highlight, kept apart from the UI theme
  deviation: tokens.colorInfo,
  warn: tokens.colorWarning,
  formation: tokens.colorTextSecondary,
  formationLine: tokens.colorTextQuaternary,
  formationHover: tokens.colorText,
  leg: tokens.colorFillQuaternary,
};

/** Pointer within this many pixels of a formation mark picks it. */
const MARK_HIT_PX = 8;

const PAD = { left: 56, right: 16, top: 14, bottom: 22 };

function niceStep(span: number, target: number): number {
  const raw = span / target;
  const mag = 10 ** Math.floor(Math.log10(raw));
  return [1, 2, 5, 10].map((m) => m * mag).find((s) => s >= raw) ?? 10 * mag;
}

export function SeparationStrip({
  separation,
  time,
  t0,
  t1,
  floor,
  floorLabel = "required",
  nominal,
  span,
  deviation,
  marks,
  timeline,
  onSeek,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [size, setSize] = useState({ w: 0, h: 0 });
  const [hover, setHover] = useState<number | null>(null);
  const dragging = useRef(false);

  const yMax = useMemo(() => {
    const ref = Math.max(floor ?? 0, nominal ?? 0, 1);
    return ref * 2.5; // the interesting band is near the floor; larger gaps clip to the top
  }, [floor, nominal]);

  useEffect(() => {
    const canvas = canvasRef.current!;
    const ro = new ResizeObserver(([entry]) => {
      setSize({ w: entry.contentRect.width, h: entry.contentRect.height });
    });
    ro.observe(canvas);
    return () => ro.disconnect();
  }, []);

  const xOf = (t: number) => PAD.left + ((t - t0) / Math.max(t1 - t0, 1e-9)) * (size.w - PAD.left - PAD.right);
  const tOf = (x: number) =>
    Math.min(t1, Math.max(t0, t0 + ((x - PAD.left) / Math.max(size.w - PAD.left - PAD.right, 1)) * (t1 - t0)));
  const yOf = (d: number) => PAD.top + (1 - Math.min(d, yMax) / yMax) * (size.h - PAD.top - PAD.bottom);

  const formations = useMemo(() => timeline?.formations ?? [], [timeline]);
  // The formation under the pointer, if any: its mark lights up and the readout explains it.
  const hoverMark: FormationMark | null = useMemo(() => {
    if (hover === null || !size.w) return null;
    let best: FormationMark | null = null;
    let bestPx = MARK_HIT_PX;
    const pxPerSec = (size.w - PAD.left - PAD.right) / Math.max(t1 - t0, 1e-9);
    for (const f of formations) {
      const px = Math.abs(f.reached_sec - hover) * pxPerSec;
      if (px <= bestPx) {
        best = f;
        bestPx = px;
      }
    }
    return best;
  }, [hover, formations, size.w, t0, t1]);

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
    ctx.textBaseline = "middle";

    // Takeoff and return legs: a faint band, labelled at the bottom.
    const legs = timeline?.legs ?? {};
    for (const [name, leg] of [["Takeoff", legs.takeoff], ["Return", legs.return]] as const) {
      if (!leg) continue;
      const x0 = xOf(Math.max(t0, leg.start_sec));
      const x1 = xOf(Math.min(t1, leg.end_sec));
      if (x1 <= x0) continue;
      ctx.fillStyle = COLORS.leg;
      ctx.fillRect(x0, PAD.top, x1 - x0, size.h - PAD.top - PAD.bottom);
      ctx.fillStyle = COLORS.formation;
      ctx.textAlign = "center";
      ctx.textBaseline = "bottom";
      if (ctx.measureText(name).width + 8 < x1 - x0) ctx.fillText(name, (x0 + x1) / 2, size.h - PAD.bottom - 4);
    }

    if (span) {
      ctx.fillStyle = span.tone === "info" ? COLORS.spanInfo : COLORS.span;
      const x0 = xOf(span.start);
      ctx.fillRect(x0, PAD.top, xOf(span.end) - x0, size.h - PAD.top - PAD.bottom);
    }

    // Distance gridlines.
    const dStep = niceStep(yMax, 3);
    ctx.fillStyle = COLORS.axis;
    ctx.textAlign = "right";
    for (let d = 0; d <= yMax + 1e-9; d += dStep) {
      const y = yOf(d);
      ctx.strokeStyle = COLORS.grid;
      ctx.beginPath();
      ctx.moveTo(PAD.left, y);
      ctx.lineTo(size.w - PAD.right, y);
      ctx.stroke();
      ctx.fillText(`${d.toFixed(dStep < 1 ? 1 : 0)} m`, PAD.left - 8, y);
    }

    // Time ticks.
    const tStep = niceStep(t1 - t0, Math.max(2, Math.floor(size.w / 110)));
    ctx.textAlign = "center";
    ctx.textBaseline = "alphabetic";
    for (let t = Math.ceil(t0 / tStep) * tStep; t <= t1 + 1e-9; t += tStep) {
      ctx.fillText(formatTime(t).replace(/\.0$/, ""), xOf(t), size.h - 6);
    }

    // Closest-pair line; below the floor it turns red and thick.
    const { times, min_m } = separation;
    ctx.lineJoin = "round";
    for (let i = 1; i < times.length; i++) {
      const a = min_m[i - 1];
      const b = min_m[i];
      if (a === null || b === null) continue;
      const bad = floor !== null && (a < floor || b < floor);
      ctx.strokeStyle = bad ? COLORS.bad : COLORS.line;
      ctx.lineWidth = bad ? 2.5 : 1.5;
      ctx.beginPath();
      ctx.moveTo(xOf(times[i - 1]), yOf(a));
      ctx.lineTo(xOf(times[i]), yOf(b));
      ctx.stroke();
    }

    if (deviation) {
      ctx.strokeStyle = COLORS.deviation;
      ctx.lineWidth = 1.25;
      ctx.beginPath();
      deviation.forEach((d, i) => (i ? ctx.lineTo(xOf(times[i]), yOf(d)) : ctx.moveTo(xOf(times[i]), yOf(d))));
      ctx.stroke();
    }

    // A label that would run into the previous one goes on the next row.
    const rowEnds: number[] = [];
    for (const m of marks ?? []) {
      if (m.time < t0 || m.time > t1) continue;
      const x = xOf(m.time);
      const w = ctx.measureText(m.label).width;
      // A label that would run off the right edge goes to the left of its line.
      const left = x + 4 + w > size.w - PAD.right ? x - 4 - w : x + 4;
      let row = rowEnds.findIndex((end) => left >= end);
      if (row < 0) row = rowEnds.length;
      rowEnds[row] = left + w + 8;
      ctx.strokeStyle = m.tone === "bad" ? COLORS.bad : COLORS.warn;
      ctx.lineWidth = 1;
      ctx.setLineDash(m.tone === "bad" ? [] : [3, 3]);
      ctx.beginPath();
      ctx.moveTo(x, PAD.top);
      ctx.lineTo(x, size.h - PAD.bottom);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = ctx.strokeStyle;
      ctx.textAlign = "left";
      ctx.textBaseline = "top";
      ctx.fillText(m.label, left, PAD.top + row * 14);
    }

    // Formation marks: a dotted line where the show reaches each formation, a bar while it is held,
    // and the name along the bottom (stacked upwards when names would overlap).
    const formationRows: number[] = [];
    ctx.font = "11px Inter, system-ui, 'Segoe UI', sans-serif";
    for (const f of formations) {
      if (f.reached_sec < t0 - 1e-6 || f.reached_sec > t1 + 1e-6) continue;
      const x = xOf(f.reached_sec);
      const lit = hoverMark === f;
      const color = f.rejected ? COLORS.bad : lit ? COLORS.formationHover : COLORS.formation;
      ctx.strokeStyle = lit ? COLORS.formationHover : f.rejected ? COLORS.bad : COLORS.formationLine;
      ctx.lineWidth = 1;
      ctx.setLineDash([2, 3]);
      ctx.beginPath();
      ctx.moveTo(x, PAD.top);
      ctx.lineTo(x, size.h - PAD.bottom);
      ctx.stroke();
      ctx.setLineDash([]);
      if (f.leaves_sec !== null && f.leaves_sec - f.reached_sec > 0.05) {
        ctx.fillStyle = color;
        ctx.fillRect(x, size.h - PAD.bottom - 2, xOf(Math.min(t1, f.leaves_sec)) - x, 2);
      }
      ctx.fillStyle = color;
      ctx.beginPath();
      const yb = size.h - PAD.bottom;
      ctx.moveTo(x, yb - 4);
      ctx.lineTo(x + 3.5, yb);
      ctx.lineTo(x, yb + 4);
      ctx.lineTo(x - 3.5, yb);
      ctx.closePath();
      ctx.fill();
      // A name that would run off the right edge goes to the left of its mark.
      const w = ctx.measureText(f.name).width;
      const left = x + 4 + w > size.w - PAD.right ? x - 4 - w : x + 4;
      let row = formationRows.findIndex((end) => left >= end);
      if (row < 0) row = formationRows.length;
      formationRows[row] = left + w + 6;
      ctx.textAlign = "left";
      ctx.textBaseline = "bottom";
      ctx.fillText(f.name, left, size.h - PAD.bottom - 4 - row * 13);
    }
    ctx.font = "12px Inter, system-ui, 'Segoe UI', sans-serif";

    if (floor !== null) {
      ctx.setLineDash([5, 4]);
      ctx.strokeStyle = COLORS.floor;
      ctx.lineWidth = 1;
      const y = yOf(floor);
      ctx.beginPath();
      ctx.moveTo(PAD.left, y);
      ctx.lineTo(size.w - PAD.right, y);
      ctx.stroke();
      ctx.setLineDash([]);
      ctx.fillStyle = COLORS.floor;
      ctx.textAlign = "right";
      ctx.textBaseline = "bottom";
      ctx.fillText(`${floor.toFixed(2)} m ${floorLabel}`, size.w - PAD.right - 4, y - 3);
    }

    const worst = separation.worst;
    if (worst.distance_m !== null) {
      const x = xOf(worst.time_sec);
      const y = yOf(worst.distance_m);
      ctx.fillStyle = floor !== null && worst.distance_m < floor ? COLORS.bad : COLORS.line;
      ctx.beginPath();
      ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fill();
    }

    const px = xOf(time);
    ctx.strokeStyle = COLORS.playhead;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(px, PAD.top - 4);
    ctx.lineTo(px, size.h - PAD.bottom);
    ctx.stroke();
  }, [size, separation, time, t0, t1, floor, floorLabel, yMax, span, deviation, marks, timeline, hoverMark]);

  const readoutTime = hover ?? time;
  const k = nearestFrame(separation.times, readoutTime);
  const readout = separation.min_m[k];

  const seekFromEvent = (e: React.PointerEvent) => {
    const rect = e.currentTarget.getBoundingClientRect();
    onSeek(tOf(e.clientX - rect.left));
  };

  const frameStep = 1 / separation.sampled_fps;
  const onKeyDown = (e: React.KeyboardEvent) => {
    const step = e.shiftKey ? 1 : frameStep;
    if (e.key === "ArrowRight") onSeek(Math.min(t1, time + step));
    else if (e.key === "ArrowLeft") onSeek(Math.max(t0, time - step));
    else if (e.key === "Home") onSeek(t0);
    else if (e.key === "End") onSeek(t1);
    else return;
    e.preventDefault();
  };

  const below = floor !== null && readout !== null && readout < floor;
  return (
    <div className="strip">
      <div className="strip-readout" aria-live="off">
        <span className="strip-time">{formatTime(readoutTime)}</span>
        <span>
          Closest pair{" "}
          <strong className={below ? "tone-bad" : "tone-ok"}>{metres(readout, 2)}</strong>
          {readout !== null && (
            <span className="strip-pair">
              {" "}
              drones {separation.a[k]} and {separation.b[k]}
            </span>
          )}
        </span>
        {deviation && (
          <span>
            <span className="strip-key strip-key-deviation" aria-hidden="true" />
            Furthest from plan <strong>{metres(deviation[k], 2)}</strong>
            <span className="strip-pair"> drone {separation.deviation_drone?.[k]}</span>
          </span>
        )}
        {hoverMark && <span className={`strip-formation ${hoverMark.rejected ? "tone-bad" : ""}`}>{formationDetail(hoverMark)}</span>}
        {span && !hoverMark && <span className={`strip-window ${span.tone === "info" ? "strip-window-info" : ""}`}>{span.label}</span>}
      </div>
      <canvas
        ref={canvasRef}
        className="strip-canvas"
        tabIndex={0}
        role="slider"
        aria-label="Show time"
        aria-valuemin={t0}
        aria-valuemax={t1}
        aria-valuenow={Number(time.toFixed(2))}
        aria-valuetext={`${formatTime(time)}, closest pair ${metres(separation.min_m[nearestFrame(separation.times, time)], 2)}`}
        onKeyDown={onKeyDown}
        onPointerDown={(e) => {
          dragging.current = true;
          e.currentTarget.setPointerCapture(e.pointerId);
          seekFromEvent(e);
        }}
        onPointerMove={(e) => {
          const rect = e.currentTarget.getBoundingClientRect();
          setHover(tOf(e.clientX - rect.left));
          if (dragging.current) seekFromEvent(e);
        }}
        onPointerUp={() => (dragging.current = false)}
        onPointerLeave={() => setHover(null)}
      />
    </div>
  );
}
