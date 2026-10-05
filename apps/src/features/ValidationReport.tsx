// Result of `studio_bridge validate`.

import type { Validation } from "../bridge/types";

const fmt = (v: number, d = 1) => v.toFixed(d);

/** Holding-area capacity: one block per layer the area may stack, filled by
 *  the drones parked in it, so a crowded or overflowing area shows before a long Stage 2 run. */
function Capacity({ summary }: { summary: NonNullable<Validation["summary"]> }) {
  const ha = summary.holding_area;
  const c = ha.capacity;
  const n = summary.fleet_size;
  const floor = ha.gatekeeper_floor_m;
  const tooClose = floor !== null && ha.grid_spacing_m < floor;
  return (
    <section className="capacity" aria-labelledby="capacity-title">
      <h3 id="capacity-title">Holding area</h3>
      <div className="capacity-gauge" role="img" aria-label={`${Math.min(n, c.capacity)} of ${c.capacity} places used`}>
        {c.layer_capacities.map((size, layer) => {
          const below = c.layer_capacities.slice(0, layer).reduce((a, b) => a + b, 0);
          const fill = Math.max(0, Math.min(1, (n - below) / size));
          return (
            <span key={layer} className="capacity-layer" title={`Layer ${layer + 1}`}>
              <span
                className={c.widened ? "capacity-fill capacity-over" : "capacity-fill"}
                style={{ width: `${fill * 100}%` }}
              />
            </span>
          );
        })}
      </div>
      <p className={c.widened ? "tone-warn" : undefined}>
        {c.widened
          ? `Too small: room for ${c.capacity} drones, the fleet has ${n}. Phase 1 widened it to ${fmt(c.width_used_m)} m.`
          : `Room for ${c.capacity} drones in ${ha.size[0]} × ${ha.size[1]} m; this show parks ${n} in ${c.layers_used} of ${c.max_layers} ${
              c.max_layers === 1 ? "layer" : "layers"
            }.`}
      </p>
      <p className="muted small">
        {c.staggered_layers
          ? `${c.layer_capacities[0]} / ${c.layer_capacities[1] ?? c.layer_capacities[0]} places per layer (alternate layers shifted half a place)`
          : `${c.per_layer} places per layer`}{" "}
        at {ha.grid_spacing_m} m apart, layers {c.layer_spacing_m} m apart up to {ha.max_height} m.{" "}
        {floor !== null && (
          <span className={tooClose ? "tone-bad" : undefined}>
            {tooClose
              ? `Parked drones start closer than the ${floor} m the safety check requires.`
              : `The safety check requires ${floor} m, so parked neighbours start clear of it.`}
          </span>
        )}
      </p>
    </section>
  );
}

/** Waiting areas: where spare drones wait in the air instead of going home. */
function WaitingAreas({ summary }: { summary: NonNullable<Validation["summary"]> }) {
  const areas = summary.waiting_areas ?? [];
  if (!areas.length) return null;
  const slots = areas.reduce((sum, a) => sum + a.slot_count, 0);
  const spare = areas[0].spare_max;
  return (
    <section className="capacity" aria-labelledby="waiting-title">
      <h3 id="waiting-title">Waiting areas</h3>
      <p>
        {areas.length} {areas.length === 1 ? "area" : "areas"} with {slots} places; at most {spare} spare{" "}
        {spare === 1 ? "drone waits" : "drones wait"} there at once, instead of flying home.
      </p>
      <ul className="muted small">
        {areas.map((a, i) => (
          <li key={i}>
            Area {i + 1}: {a.slot_count} places at ({a.center.map((v) => fmt(v, 0)).join(", ")}), {a.size[0]} × {a.size[1]} m,{" "}
            {fmt(a.center[2], 0)} m up
          </li>
        ))}
      </ul>
    </section>
  );
}

export function ValidationReport({ validation }: { validation: Validation }) {
  const { ok, errors, warnings, summary } = validation;
  return (
    <div className="validation">
      {!ok && (
        <section className="issues issues-bad">
          <h3>
            {errors.length} {errors.length === 1 ? "problem" : "problems"} to fix in the file
          </h3>
          <p className="muted">Stage 2 can't read this file until these are fixed in the Blender export.</p>
          <ul>
            {errors.map((e, i) => (
              <li key={i}>
                <code>{e.path}</code>
                <span>{e.message}</span>
              </li>
            ))}
          </ul>
        </section>
      )}

      {warnings.length > 0 && (
        <section className="issues issues-warn">
          <h3>{warnings.length === 1 ? "One thing to check" : `${warnings.length} things to check`}</h3>
          <ul>
            {warnings.map((w, i) => (
              <li key={i}>
                <code>{w.path}</code>
                <span>{w.message}</span>
              </li>
            ))}
          </ul>
        </section>
      )}

      {summary && (
        <>
          <dl className="stat-cards">
            <div>
              <dt>Drones</dt>
              <dd>{summary.fleet_size}</dd>
            </div>
            <div>
              <dt>Formations</dt>
              <dd>{summary.keyframes.length}</dd>
            </div>
            <div>
              <dt>Designed length</dt>
              <dd>{fmt(summary.total_duration_sec)} s</dd>
            </div>
            <div>
              <dt>Minimum spacing</dt>
              <dd>{summary.min_distance_m} m</dd>
            </div>
          </dl>
          <Capacity summary={summary} />
          <WaitingAreas summary={summary} />
          <h3>Formations, in show order</h3>
          <ol className="formation-cards">
            {summary.keyframes.map((k, i) => {
              const tight = k.min_spacing_m < summary.min_distance_m - 1e-6;
              return (
                <li key={i}>
                  <span className="formation-num">{String(i + 1).padStart(2, "0")}</span>
                  <span className="formation-name">{k.shape_name}</span>
                  <span className="muted small">
                    {k.points} points at {fmt(k.time_sec)} s
                  </span>
                  <span className={`small ${tight ? "tone-bad" : "muted"}`}>Closest points {fmt(k.min_spacing_m, 2)} m</span>
                </li>
              );
            })}
          </ol>
        </>
      )}
    </div>
  );
}
