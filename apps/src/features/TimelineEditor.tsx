// Weather timeline editor of the Conditions tab: one lane per
// channel over show time, aligned with the show's formations. Click a lane to add a key, drag a key
// to move it (wind and rain also up and down to change the value), arrow keys nudge the focused key.

import { useEffect, useMemo, useRef, useState } from "react";
import type { ConditionsInfo, RtkState, Scenario } from "../bridge/types";
import { formatTime } from "../replay/sampling";
import { compass } from "../replay/weather";

export type Channel = "wind" | "gusts" | "rtk" | "rain";
export interface KeyRef {
  channel: Channel;
  index: number;
}

interface Props {
  scenario: Scenario;
  show: ConditionsInfo["show"];
  /** Seconds the timeline covers (the show plus the simulation's settling tail). */
  duration: number;
  selected: KeyRef | null;
  onSelect: (key: KeyRef | null) => void;
  onChange: (next: Scenario, select?: KeyRef | null) => void;
  disabled?: boolean;
}

export const LABEL_W = 92;
export const PAD_R = 16;
const AXIS_H = 22;
const LANES: { id: Channel | "show"; label: string; h: number }[] = [
  { id: "wind", label: "Wind", h: 84 },
  { id: "gusts", label: "Gusts", h: 44 },
  { id: "rtk", label: "RTK", h: 30 },
  { id: "rain", label: "Rain", h: 72 },
  { id: "show", label: "Show", h: 34 },
];
const GAP = 10;
const SNAP_S = 0.5;
const NEXT_RTK: Record<RtkState, RtkState> = { fixed: "float", float: "gps", gps: "fixed" };

const round = (v: number, step: number) => Math.round(v / step) * step;
const clamp = (v: number, lo: number, hi: number) => Math.min(hi, Math.max(lo, v));

function niceStep(span: number, target: number): number {
  const raw = span / Math.max(target, 1);
  const mag = 10 ** Math.floor(Math.log10(raw));
  return [1, 2, 5, 10].map((m) => m * mag).find((s) => s >= raw) ?? 10 * mag;
}

/** Linear value of `name` at t, holding the first / last key outside them. */
function valueAt<K extends { t: number }>(keys: K[], t: number, get: (k: K) => number, fallback: number): number {
  if (!keys.length) return fallback;
  const sorted = [...keys].sort((a, b) => a.t - b.t);
  if (t <= sorted[0].t) return get(sorted[0]);
  for (let i = 1; i < sorted.length; i++) {
    if (t <= sorted[i].t) {
      const a = sorted[i - 1];
      const b = sorted[i];
      return b.t === a.t ? get(b) : get(a) + ((get(b) - get(a)) * (t - a.t)) / (b.t - a.t);
    }
  }
  return get(sorted[sorted.length - 1]);
}

function directionAt(scenario: Scenario, t: number): number {
  const keys = [...scenario.wind].sort((a, b) => a.t - b.t);
  if (!keys.length) return 270;
  if (t <= keys[0].t) return keys[0].from_deg;
  for (let i = 1; i < keys.length; i++) {
    if (t <= keys[i].t) {
      const a = keys[i - 1];
      const b = keys[i];
      const d = ((b.from_deg - a.from_deg + 540) % 360) - 180;
      const f = b.t === a.t ? 1 : (t - a.t) / (b.t - a.t);
      return (a.from_deg + d * f + 360) % 360;
    }
  }
  return keys[keys.length - 1].from_deg;
}

function rtkAt(scenario: Scenario, t: number): RtkState {
  const keys = [...scenario.rtk].sort((a, b) => a.t - b.t);
  let state: RtkState = keys[0]?.state ?? "fixed";
  for (const k of keys) if (k.t <= t) state = k.state;
  return state;
}

export function TimelineEditor({ scenario, show, duration, selected, onSelect, onChange, disabled }: Props) {
  const hostRef = useRef<HTMLDivElement>(null);
  const [width, setWidth] = useState(800);
  const drag = useRef<{ key: KeyRef; moved: boolean } | null>(null);

  useEffect(() => {
    const ro = new ResizeObserver(([e]) => setWidth(Math.max(320, e.contentRect.width)));
    ro.observe(hostRef.current!);
    return () => ro.disconnect();
  }, []);

  const plotW = width - LABEL_W - PAD_R;
  const x = (t: number) => LABEL_W + (clamp(t, 0, duration) / Math.max(duration, 1e-6)) * plotW;
  const tOf = (px: number) => clamp(round(((px - LABEL_W) / plotW) * duration, SNAP_S), 0, duration);

  const tops = useMemo(() => {
    let y = AXIS_H;
    const out: Record<string, number> = {};
    for (const lane of LANES) {
      out[lane.id] = y;
      y += lane.h + GAP;
    }
    out.total = y;
    return out;
  }, []);
  const laneH = (id: string) => LANES.find((l) => l.id === id)!.h;

  const windMax = Math.max(12, Math.ceil(Math.max(0, ...scenario.wind.map((k) => k.speed_mps)) / 4) * 4 + 2);
  const rule = scenario.rain_rule;
  const rainMax = Math.max(5, rule.limit_mm_h * 1.6, Math.ceil(Math.max(0, ...scenario.rain.map((k) => k.mm_h)) * 1.15));
  const yWind = (v: number) => tops.wind + 6 + (1 - v / windMax) * (laneH("wind") - 12);
  const yRain = (v: number) => tops.rain + 4 + (1 - v / rainMax) * (laneH("rain") - 8);
  const windOfY = (py: number) => clamp(round((1 - (py - tops.wind - 6) / (laneH("wind") - 12)) * windMax, 0.1), 0, 40);
  const rainOfY = (py: number) => clamp(round((1 - (py - tops.rain - 4) / (laneH("rain") - 8)) * rainMax, 0.1), 0, 200);

  // ---- editing ----
  function withKeys(channel: Channel, keys: Scenario[Channel], select?: KeyRef | null) {
    onChange({ ...scenario, [channel]: keys }, select);
  }

  function addKey(channel: Channel, t: number, py: number) {
    const keys = scenario[channel] as { t: number }[];
    if (keys.some((k) => Math.abs(k.t - t) < SNAP_S / 2)) return;
    let key: Scenario[Channel][number];
    if (channel === "wind") {
      key = {
        t,
        speed_mps: windOfY(py),
        from_deg: Math.round(directionAt(scenario, t)),
        turbulence: round(valueAt(scenario.wind, t, (k) => k.turbulence, 0.15), 0.01),
      };
    } else if (channel === "gusts") {
      key = { t, peak_mps: 4, duration_s: 4, from_deg: Math.round(directionAt(scenario, t)) };
    } else if (channel === "rtk") {
      key = { t, state: NEXT_RTK[rtkAt(scenario, t)] };
    } else {
      key = { t, mm_h: rainOfY(py) };
    }
    const next = [...keys, key].sort((a, b) => a.t - b.t);
    withKeys(channel, next as Scenario[Channel], { channel, index: next.indexOf(key) });
  }

  function moveKey(ref: KeyRef, t: number, py: number | null) {
    const keys = [...(scenario[ref.channel] as { t: number }[])];
    const key = { ...keys[ref.index], t } as Record<string, unknown> & { t: number };
    if (py !== null && ref.channel === "wind") key.speed_mps = windOfY(py);
    if (py !== null && ref.channel === "rain") key.mm_h = rainOfY(py);
    keys[ref.index] = key;
    // Keep keys in time order and the selection on the moved key.
    const order = keys.map((k, i) => ({ k, i })).sort((a, b) => a.k.t - b.k.t);
    const index = order.findIndex((o) => o.i === ref.index);
    withKeys(ref.channel, order.map((o) => o.k) as Scenario[Channel], { channel: ref.channel, index });
    return index;
  }

  const local = (e: React.PointerEvent) => {
    const r = (e.currentTarget as SVGElement).ownerSVGElement?.getBoundingClientRect() ??
      (e.currentTarget as SVGElement).getBoundingClientRect();
    return { px: e.clientX - r.left, py: e.clientY - r.top };
  };

  const onLaneDown = (channel: Channel) => (e: React.PointerEvent<SVGRectElement>) => {
    if (disabled || e.button !== 0) return;
    const { px, py } = local(e);
    addKey(channel, tOf(px), py);
  };

  const onKeyDown = (ref: KeyRef) => (e: React.PointerEvent<SVGElement>) => {
    if (disabled || e.button !== 0) return;
    e.stopPropagation();
    (e.currentTarget as SVGElement).setPointerCapture(e.pointerId);
    drag.current = { key: ref, moved: false };
    onSelect(ref);
  };

  const onKeyMove = (e: React.PointerEvent<SVGElement>) => {
    const d = drag.current;
    if (!d) return;
    const { px, py } = local(e);
    const vertical = d.key.channel === "wind" || d.key.channel === "rain";
    d.key = { ...d.key, index: moveKey(d.key, tOf(px), vertical ? py : null) };
    d.moved = true;
  };

  const onKeyUp = () => {
    drag.current = null;
  };

  const onKeyboard = (ref: KeyRef) => (e: React.KeyboardEvent) => {
    if (disabled) return;
    const keys = scenario[ref.channel] as { t: number }[];
    const k = keys[ref.index] as Record<string, number> & { t: number };
    const step = e.shiftKey ? 5 : SNAP_S;
    if (e.key === "ArrowLeft" || e.key === "ArrowRight") {
      moveKey(ref, clamp(k.t + (e.key === "ArrowRight" ? step : -step), 0, duration), null);
    } else if ((e.key === "ArrowUp" || e.key === "ArrowDown") && (ref.channel === "wind" || ref.channel === "rain")) {
      const field = ref.channel === "wind" ? "speed_mps" : "mm_h";
      const next = [...keys] as Record<string, number>[];
      next[ref.index] = { ...k, [field]: clamp(round(k[field] + (e.key === "ArrowUp" ? 0.5 : -0.5), 0.1), 0, 200) };
      withKeys(ref.channel, next as unknown as Scenario[Channel], ref);
    } else if (e.key === "Delete" || e.key === "Backspace") {
      withKeys(ref.channel, keys.filter((_, i) => i !== ref.index) as Scenario[Channel], null);
    } else return;
    e.preventDefault();
  };

  const isSel = (channel: Channel, index: number) => selected?.channel === channel && selected.index === index;
  const keyProps = (channel: Channel, index: number, label: string) => ({
    tabIndex: disabled ? -1 : 0,
    role: "button",
    "aria-label": label,
    "aria-pressed": isSel(channel, index),
    className: `tl-key tl-key-${channel}${isSel(channel, index) ? " is-selected" : ""}`,
    onPointerDown: onKeyDown({ channel, index }),
    onPointerMove: onKeyMove,
    onPointerUp: onKeyUp,
    onFocus: () => onSelect({ channel, index }),
    onKeyDown: onKeyboard({ channel, index }),
  });

  // ---- drawing helpers ----
  /** Polyline through keys, held flat to both ends of the timeline. */
  function linePath(keys: { t: number; v: number }[], y: (v: number) => number, fallback: number): string {
    const pts = keys.length ? keys : [{ t: 0, v: fallback }];
    const all = [{ t: 0, v: pts[0].v }, ...pts, { t: duration, v: pts[pts.length - 1].v }];
    return all.map((p, i) => `${i ? "L" : "M"}${x(p.t).toFixed(1)},${y(p.v).toFixed(1)}`).join(" ");
  }

  const windPts = scenario.wind.map((k) => ({ t: k.t, v: k.speed_mps }));
  const rainPts = scenario.rain.map((k) => ({ t: k.t, v: k.mm_h }));
  const windLine = linePath(windPts, yWind, 0);
  const rainLine = linePath(rainPts, yRain, 0);
  const windBase = yWind(0);
  const rainBase = yRain(0);

  const tStep = niceStep(duration, Math.max(2, Math.floor(plotW / 90)));
  const ticks: number[] = [];
  for (let t = 0; t <= duration + 1e-9; t += tStep) ticks.push(t);

  // Direction arrows along the wind lane, about every 70 px.
  const arrowCount = Math.max(2, Math.floor(plotW / 70));
  const arrows = Array.from({ length: arrowCount }, (_, i) => ((i + 0.5) / arrowCount) * duration);

  // RTK bands from the keys (the first state holds before the first key).
  const rtkKeys = [...scenario.rtk].sort((a, b) => a.t - b.t);
  const rtkBands = rtkKeys.length
    ? rtkKeys.map((k, i) => ({ from: i === 0 ? 0 : k.t, to: rtkKeys[i + 1]?.t ?? duration, state: k.state }))
    : [{ from: 0, to: duration, state: "fixed" as RtkState }];

  // Show lane: holds at formations between transitions, transitions as thin connectors.
  const holds = show.transitions.slice(0, -1).map((tr, i) => ({
    from: tr.end_time_sec,
    to: show.transitions[i + 1].start_time_sec,
    name: tr.to_keyframe,
  }));
  if (show.transitions.length) {
    const last = show.transitions[show.transitions.length - 1];
    if (last.to_keyframe !== "holding_area") holds.push({ from: last.end_time_sec, to: show.duration_sec, name: last.to_keyframe });
  }

  // Formation names where each formation is reached; a name that would run into the next one is left
  // out (its block still has a tooltip).
  const CHAR_W = 6.6;
  const showLabels: { key: string; x: number; text: string; anchor: "start" | "end" }[] = [];
  const named = show.transitions.filter((tr) => tr.to_keyframe !== "holding_area");
  named.forEach((tr, i) => {
    const left = x(tr.end_time_sec) + 3;
    const next = named[i + 1] ? x(named[i + 1].end_time_sec) : LABEL_W + plotW - 50;
    if (left + tr.to_keyframe.length * CHAR_W + 6 <= next) {
      showLabels.push({ key: `n${tr.index}`, x: left, text: tr.to_keyframe, anchor: "start" });
    }
  });
  const landing = show.transitions.find((tr) => tr.to_keyframe === "holding_area");
  if (landing) showLabels.push({ key: "landed", x: x(landing.end_time_sec) - 3, text: "landed", anchor: "end" });

  return (
    <div className="timeline" ref={hostRef}>
      <svg width={width} height={tops.total} className="tl-svg" role="group" aria-label="Weather timeline">
        {/* time axis */}
        {ticks.map((t) => (
          <g key={t}>
            <line x1={x(t)} x2={x(t)} y1={AXIS_H - 4} y2={tops.total - GAP} className="tl-grid" />
            <text x={x(t)} y={AXIS_H - 8} className="tl-tick" textAnchor="middle">
              {formatTime(t).replace(/\.0$/, "")}
            </text>
          </g>
        ))}
        {show.duration_sec < duration && (
          <rect x={x(show.duration_sec)} y={AXIS_H} width={x(duration) - x(show.duration_sec)} height={tops.total - AXIS_H - GAP} className="tl-tail">
            <title>After the show: the drones settle on the ground</title>
          </rect>
        )}

        {LANES.map((lane) => (
          <g key={lane.id}>
            <text x={0} y={tops[lane.id] + lane.h / 2} className="tl-lane-label" dominantBaseline="middle">
              {lane.label}
            </text>
            <rect
              x={LABEL_W}
              y={tops[lane.id]}
              width={plotW}
              height={lane.h}
              className={`tl-lane${lane.id === "show" || disabled ? "" : " tl-lane-edit"}`}
              onPointerDown={lane.id === "show" ? undefined : onLaneDown(lane.id as Channel)}
            />
          </g>
        ))}

        {/* wind: speed line, direction arrows, keys */}
        <path d={`${windLine} L${x(duration)},${windBase} L${x(0)},${windBase} Z`} className="tl-area tl-area-wind" />
        <path d={windLine} className="tl-line tl-line-wind" />
        {arrows.map((t) => {
          const v = valueAt(scenario.wind, t, (k) => k.speed_mps, 0);
          if (v < 0.3) return null;
          return (
            <g key={t} transform={`translate(${x(t)},${tops.wind + laneH("wind") - 12}) rotate(${directionAt(scenario, t) + 180})`} className="tl-arrow">
              <path d="M0 -6 L3.5 2 L0 0.5 L-3.5 2 Z" />
            </g>
          );
        })}
        <text x={LABEL_W + 4} y={tops.wind + 10} className="tl-scale">
          {windMax} m/s
        </text>
        {scenario.wind.map((k, i) => (
          <circle key={i} cx={x(k.t)} cy={yWind(k.speed_mps)} r={6}
            {...keyProps("wind", i, `Wind key at ${formatTime(k.t)}: ${k.speed_mps} m/s from ${compass(k.from_deg)}`)} />
        ))}

        {/* gusts: one marker per front, as wide as it lasts, as tall as its peak */}
        {scenario.gusts.map((g, i) => {
          const h = 8 + (Math.min(g.peak_mps, 15) / 15) * (laneH("gusts") - 12);
          const base = tops.gusts + laneH("gusts") - 3;
          const x0 = x(g.t);
          const x1 = Math.max(x(g.t + g.duration_s), x0 + 8);
          return (
            <path key={i} d={`M${x0},${base} L${(x0 + x1) / 2},${base - h} L${x1},${base} Z`}
              {...keyProps("gusts", i, `Gust at ${formatTime(g.t)}: ${g.peak_mps} m/s for ${g.duration_s} s`)} />
          );
        })}

        {/* RTK: a band of states */}
        {rtkBands.map((b, i) => (
          <rect key={i} x={x(b.from)} y={tops.rtk + 4} width={Math.max(0, x(b.to) - x(b.from))} height={laneH("rtk") - 8}
            className={`tl-rtk tl-rtk-${b.state}`} pointerEvents="none" />
        ))}
        {rtkBands.map((b, i) =>
          x(b.to) - x(b.from) > 46 ? (
            <text key={`l${i}`} x={x(b.from) + 6} y={tops.rtk + laneH("rtk") / 2} className="tl-rtk-label" dominantBaseline="middle" pointerEvents="none">
              {b.state === "gps" ? "GPS only" : b.state}
            </text>
          ) : null,
        )}
        {scenario.rtk.map((k, i) => (
          <rect key={i} x={x(k.t) - 3} y={tops.rtk} width={6} height={laneH("rtk")} rx={2}
            {...keyProps("rtk", i, `RTK becomes ${k.state} at ${formatTime(k.t)}`)} />
        ))}

        {/* rain: intensity, with the alert and limit levels */}
        <path d={`${rainLine} L${x(duration)},${rainBase} L${x(0)},${rainBase} Z`} className="tl-area tl-area-rain" />
        <line x1={LABEL_W} x2={LABEL_W + plotW} y1={yRain(rule.alert_mm_h)} y2={yRain(rule.alert_mm_h)} className="tl-level tl-level-alert" />
        <line x1={LABEL_W} x2={LABEL_W + plotW} y1={yRain(rule.limit_mm_h)} y2={yRain(rule.limit_mm_h)} className="tl-level tl-level-limit" />
        <text x={LABEL_W + plotW - 4} y={yRain(rule.alert_mm_h) - 3} className="tl-level-label tl-level-label-alert" textAnchor="end">
          alert {rule.alert_mm_h} mm/h
        </text>
        <text x={LABEL_W + plotW - 4} y={yRain(rule.limit_mm_h) - 3} className="tl-level-label tl-level-label-limit" textAnchor="end">
          limit {rule.limit_mm_h} mm/h
        </text>
        <path d={rainLine} className="tl-line tl-line-rain" />
        {scenario.rain.map((k, i) => (
          <circle key={i} cx={x(k.t)} cy={yRain(k.mm_h)} r={6}
            {...keyProps("rain", i, `Rain key at ${formatTime(k.t)}: ${k.mm_h} mm/h`)} />
        ))}

        {/* show: formations held, transitions between them */}
        {show.transitions.map((tr) => (
          <rect key={`t${tr.index}`} x={x(tr.start_time_sec)} y={tops.show + laneH("show") / 2 - 1} width={Math.max(0, x(tr.end_time_sec) - x(tr.start_time_sec))} height={2} className="tl-transition">
            <title>
              {tr.from_keyframe === "holding_area" ? "Takeoff" : tr.from_keyframe} to {tr.to_keyframe === "holding_area" ? "the holding area" : tr.to_keyframe},{" "}
              {formatTime(tr.start_time_sec)} to {formatTime(tr.end_time_sec)}
            </title>
          </rect>
        ))}
        {holds.map((h, i) => (
          <g key={`h${i}`}>
            <rect x={x(h.from)} y={tops.show + 4} width={Math.max(2, x(h.to) - x(h.from))} height={laneH("show") - 8} rx={3} className="tl-hold">
              <title>
                {h.name}, {formatTime(h.from)} to {formatTime(h.to)}
              </title>
            </rect>
          </g>
        ))}
        {showLabels.map((l) => (
          <text key={l.key} x={l.x} y={tops.show + laneH("show") / 2} className="tl-show-label" dominantBaseline="middle"
            textAnchor={l.anchor} pointerEvents="none">
            {l.text}
          </text>
        ))}
      </svg>
    </div>
  );
}
