// 3D replay with the separation timeline as scrubber (docs/5-studio_gui.md §6.4).

import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { ReplayScene } from "./ReplayScene";
import { SeparationStrip } from "./SeparationStrip";
import { distanceAt, formatTime, metres, nearestTo, positionAt } from "./sampling";
import type { ReplayData, ReplayFocus, SimWeather } from "./types";
import { WeatherHud } from "./WeatherHud";
import { currentFormation, formationDelay, formationDetail } from "./formations";

const SPEEDS = [0.25, 0.5, 1, 2, 4, 8];

const placeName = (keyframe: string) => (keyframe === "holding_area" ? "holding area" : keyframe);

/** First sampled time at which the rain reaches `level`, or null. */
function rainReaches(w: SimWeather, level: number): number | null {
  const i = w.rain_mm_h.findIndex((v) => v >= level);
  return i < 0 ? null : i / w.hz;
}

interface Props {
  data: ReplayData;
  focus: ReplayFocus | null;
  label: string; // what this replay shows, e.g. "Rejected attempt"
  /** Rebuild the replay files; offered when they predate the formation marks. */
  onRebuild?: () => void;
  rebuilding?: boolean;
}

export function ReplayPlayer({ data, focus, label, onRebuild, rebuilding }: Props) {
  const { header, separation } = data;
  const hostRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<ReplayScene | null>(null);
  const [time, setTime] = useState(header.t0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const [highlight, setHighlight] = useState<number[]>([]);
  const timeRef = useRef(time);
  timeRef.current = time;
  const highlightRef = useRef(highlight);
  highlightRef.current = highlight;
  const panelRef = useRef<HTMLElement>(null);

  const failure = header.overlays.failure;
  const sim = header.overlays.simulation ?? null;
  // A simulated flight is judged by the crash distance, a plan by the planner's required distance.
  const floor = sim ? separation.crash_m : (header.overlays.gatekeeper_floor_m ?? null);
  const deviation = separation.deviation_m ?? null;
  const furthest = useMemo(() => {
    if (!deviation?.length) return null;
    let k = 0;
    deviation.forEach((d, i) => d > deviation[k] && (k = i));
    return { time: separation.times[k], distance: deviation[k], drone: separation.deviation_drone?.[k] ?? -1 };
  }, [deviation, separation]);
  const marks = useMemo(() => {
    if (!sim) return [];
    const out: { time: number; label: string; tone: "warn" | "bad" }[] = [];
    const home = sim.rain_return;
    const alert = sim.alert_time_sec ?? rainReaches(sim, sim.rain_rule.alert_mm_h);
    const limit = home?.deadline_sec ?? rainReaches(sim, sim.rain_rule.limit_mm_h);
    if (alert !== null) out.push({ time: alert, label: "Rain alert", tone: "warn" });
    if (home && home.command_sec > (alert ?? -1) + 0.5) out.push({ time: home.command_sec, label: "Return called", tone: "warn" });
    if (limit !== null) out.push({ time: limit, label: "Rain limit", tone: "bad" });
    return out;
  }, [sim]);

  const pick = useCallback((drone: number | null) => {
    if (drone === null) {
      setHighlight([]);
      return;
    }
    const near = nearestTo(data, drone, timeRef.current);
    setHighlight(near.other >= 0 ? [drone, near.other] : [drone]);
  }, [data]);

  useEffect(() => {
    const scene = new ReplayScene(hostRef.current!, data, (d) => pick(d));
    sceneRef.current = scene;
    // A rebuilt scene starts at t0; bring it to where the player already is.
    scene.setTime(timeRef.current);
    scene.setHighlight(highlightRef.current);
    const panel = panelRef.current;
    const host = hostRef.current!;
    const fit = () => {
      if (!panel) return;
      const inset = host.getBoundingClientRect().right - panel.getBoundingClientRect().left;
      scene.setRightInset(inset);
    };
    const ro = new ResizeObserver(fit);
    if (panel) ro.observe(panel);
    fit();
    return () => {
      ro.disconnect();
      scene.dispose();
      sceneRef.current = null;
    };
  }, [data, pick]);

  useEffect(() => sceneRef.current?.setTime(time), [time]);
  useEffect(() => sceneRef.current?.setHighlight(highlight), [highlight]);

  useEffect(() => {
    if (!focus) return;
    setPlaying(false);
    setTime(Math.min(header.t1, Math.max(header.t0, focus.time)));
    setHighlight(focus.drones);
    // Let the scene take the new time before centring on the pair.
    requestAnimationFrame(() => sceneRef.current?.focusOn(focus.drones));
  }, [focus, header.t0, header.t1]);

  useEffect(() => {
    if (!playing) return;
    let last = performance.now();
    let raf = requestAnimationFrame(function tick(now) {
      const next = timeRef.current + ((now - last) / 1000) * speed;
      last = now;
      if (next >= header.t1) {
        setTime(header.t1);
        setPlaying(false);
        return;
      }
      setTime(next);
      raf = requestAnimationFrame(tick);
    });
    return () => cancelAnimationFrame(raf);
  }, [playing, speed, header.t1]);

  const togglePlay = () => {
    if (!playing && time >= header.t1) setTime(header.t0);
    setPlaying((p) => !p);
  };

  const onKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === " " && (e.target as HTMLElement).tagName !== "BUTTON") {
      e.preventDefault();
      togglePlay();
    }
  };

  const worst = separation.worst;
  const pair = highlight.length === 2 ? highlight : null;
  const pairDistance = pair ? distanceAt(data, pair[0], pair[1], time) : null;
  const back = header.overlays.return_path;
  const simHome = sim?.rain_return;
  const span = failure
    ? {
        start: failure.transition.start_time_sec,
        end: failure.transition.start_time_sec + failure.transition.duration_sec,
        label: `Rejected: ${placeName(failure.transition.from_keyframe)} to ${placeName(failure.transition.to_keyframe)}`,
        tone: "bad" as const,
      }
    : back
      ? {
          start: back.abort_time_sec,
          end: back.abort_time_sec + back.duration_sec,
          label: back.inside_transition
            ? `Flight home from the move to ${back.from_keyframe} at ${formatTime(back.abort_time_sec)}`
            : `Flight home from ${back.from_keyframe}`,
          tone: "info" as const,
        }
      : simHome && simHome.formation_name && simHome.planned_home_sec !== null
        ? {
            start: simHome.start_sec,
            end: simHome.planned_home_sec,
            label: `Flight home from ${simHome.formation_name}`,
            tone: "info" as const,
          }
        : null;

  const formations = header.timeline?.formations ?? [];
  const atFormation = currentFormation(formations, time);
  const jumpTo = (t: number) => {
    setPlaying(false);
    setTime(Math.min(header.t1, Math.max(header.t0, t)));
  };

  const seekTo = (t: number, drones: number[]) => {
    setPlaying(false);
    setTime(t);
    setHighlight(drones);
    requestAnimationFrame(() => sceneRef.current?.focusOn(drones));
  };

  return (
    <div className="replay" onKeyDown={onKeyDown}>
      <div className="replay-stage">
        <div className="replay-canvas" ref={hostRef} />
        {sim && <WeatherHud weather={sim} time={time} />}
        <aside className="replay-panel" ref={panelRef}>
          <p className="replay-kind">{label}</p>
          {focus?.note && <p className="replay-note">{focus.note}</p>}
          <dl className="facts">
            <div>
              <dt>Drones</dt>
              <dd>{header.fleet_size}</dd>
            </div>
            <div>
              <dt>Closest</dt>
              <dd className={floor !== null && (worst.distance_m ?? Infinity) < floor ? "tone-bad" : "tone-ok"}>
                {metres(worst.distance_m, 3)}
              </dd>
            </div>
            <div>
              <dt>{sim ? "Crash" : "Required"}</dt>
              <dd>
                {sim && "< "}
                {metres(floor, 2)}
              </dd>
            </div>
            {furthest && (
              <div>
                <dt>Furthest from plan</dt>
                <dd className={furthest.distance > 0.5 ? "tone-warn" : undefined}>{metres(furthest.distance, 2)}</dd>
              </div>
            )}
          </dl>
          <button
            className="link"
            onClick={() => seekTo(worst.time_sec, [worst.a, worst.b])}
            disabled={worst.distance_m === null}
          >
            Go to closest approach ({formatTime(worst.time_sec)}, drones {worst.a} and {worst.b})
          </button>
          {furthest && (
            <button className="link" onClick={() => seekTo(furthest.time, [furthest.drone])}>
              Go to the furthest from plan ({formatTime(furthest.time)}, drone {furthest.drone})
            </button>
          )}
          {sim && (
            <p className="muted hint">
              Grey dots are where the plan puts each drone; a red line joins a drone more than 0.5 m from its plan.
            </p>
          )}

          {pair && (
            <section className="panel-block">
              <h3>Selected pair</h3>
              <p>
                Drones {pair[0]} and {pair[1]} are{" "}
                <strong className={floor !== null && pairDistance! < floor ? "tone-bad" : "tone-ok"}>
                  {metres(pairDistance, 2)}
                </strong>{" "}
                apart at {formatTime(time)}.
              </p>
              <p className="muted">
                Drone {pair[0]} at {positionAt(data, pair[0], time).map((v) => v.toFixed(1)).join(", ")} m
              </p>
              <button className="link" onClick={() => setHighlight([])}>
                Clear selection
              </button>
            </section>
          )}
          {!pair && <p className="muted hint">Click a drone to see its nearest neighbour.</p>}
          {formations.length > 0 && (
            <section className="panel-block">
              <h3>Formations</h3>
              <ol className="formation-jumps">
                {formations.map((f) => (
                  <li key={f.index}>
                    <button
                      className={atFormation === f ? "is-current" : undefined}
                      title={formationDetail(f)}
                      onClick={() => jumpTo(f.reached_sec)}
                    >
                      <span className={f.rejected ? "tone-bad" : undefined}>{f.name}</span>
                      <span className="clock-small">{formatTime(f.reached_sec)}</span>
                      <span className="muted">{formationDelay(f) ?? ""}</span>
                    </button>
                  </li>
                ))}
              </ol>
              <p className="muted small">Times in the planned show; the difference is from the Blender export.</p>
            </section>
          )}
          {!header.timeline && !sim && onRebuild && (
            <section className="panel-block">
              <h3>Formations</h3>
              <p className="muted small">This replay was built before formation marks existed.</p>
              <button className="link" onClick={onRebuild} disabled={rebuilding}>
                {rebuilding ? "Rebuilding the replay…" : "Rebuild the replay to show them"}
              </button>
            </section>
          )}

          {failure && (
            <section className="panel-block">
              <h3>Too-close pairs</h3>
              <ol className="violations">
                {failure.violations.slice(0, 50).map((v) => (
                  <li key={`${v.drone_a}-${v.drone_b}`}>
                    <button onClick={() => seekTo(v.time_sec, [v.drone_a, v.drone_b])}>
                      <span>
                        {v.drone_a} and {v.drone_b}
                      </span>
                      <span className="tone-bad">{metres(v.distance_m, 2)}</span>
                      <span className="muted">{formatTime(v.time_sec)}</span>
                    </button>
                  </li>
                ))}
              </ol>
            </section>
          )}

          {header.below_ground.length > 0 && (
            <section className="panel-block warn-block">
              <h3>Below ground</h3>
              <p>
                {header.below_ground.length} {header.below_ground.length === 1 ? "drone goes" : "drones go"} below
                z = {header.ground_z_m} m. Stage 2 keeps paths above the ground only when the show file declares
                one (Blender add-on 1.6.0 or later); re-export the show to apply it.
              </p>
              <ol className="violations">
                {header.below_ground.slice(0, 20).map((g) => (
                  <li key={g.drone}>
                    <button onClick={() => seekTo(g.time_sec, [g.drone])}>
                      <span>Drone {g.drone}</span>
                      <span className="tone-warn">{metres(g.min_z_m, 2)}</span>
                      <span className="muted">{formatTime(g.time_sec)}</span>
                    </button>
                  </li>
                ))}
              </ol>
            </section>
          )}
        </aside>
      </div>

      <div className="transport">
        <button className="play" onClick={togglePlay} aria-label={playing ? "Pause" : "Play"}>
          {playing ? (
            <svg viewBox="0 0 16 16" aria-hidden="true">
              <rect x="3" y="2" width="3.5" height="12" />
              <rect x="9.5" y="2" width="3.5" height="12" />
            </svg>
          ) : (
            <svg viewBox="0 0 16 16" aria-hidden="true">
              <path d="M4 2 L14 8 L4 14 Z" />
            </svg>
          )}
        </button>
        <label className="speed">
          <span className="visually-hidden">Playback speed</span>
          <select value={speed} onChange={(e) => setSpeed(Number(e.target.value))}>
            {SPEEDS.map((s) => (
              <option key={s} value={s}>
                {s}×
              </option>
            ))}
          </select>
        </label>
        <span className="clock">
          {formatTime(time)} <span className="muted">of {formatTime(header.t1)}</span>
        </span>
        <button className="link" onClick={() => sceneRef.current?.frameAll()}>
          Reset view
        </button>
      </div>
      <SeparationStrip
        separation={separation}
        time={time}
        t0={header.t0}
        t1={header.t1}
        floor={floor}
        nominal={header.overlays.nominal_min_distance_m ?? null}
        span={span}
        floorLabel={sim ? "crash" : "required"}
        deviation={deviation}
        marks={marks}
        timeline={header.timeline ?? null}
        onSeek={(t) => {
          setPlaying(false);
          setTime(t);
        }}
      />
    </div>
  );
}
