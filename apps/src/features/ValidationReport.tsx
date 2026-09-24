// Result of `studio_bridge validate` (docs/5-studio_gui.md §6.1).

import type { Validation } from "../bridge/types";

const fmt = (v: number, d = 1) => v.toFixed(d);

/** Holding-area capacity (docs/5-studio_gui.md §6.1): one block per layer the area may stack, filled by
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
        {Array.from({ length: c.max_layers }, (_, layer) => {
          const fill = Math.max(0, Math.min(1, (n - layer * c.per_layer) / c.per_layer));
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
        {c.per_layer} places per layer at {ha.grid_spacing_m} m apart, layers stacked up to {ha.max_height} m.{" "}
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
          <dl className="facts facts-wide">
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
          <table className="table">
            <caption className="visually-hidden">Formations</caption>
            <thead>
              <tr>
                <th scope="col">Formation</th>
                <th scope="col" className="num">At</th>
                <th scope="col" className="num">Points</th>
                <th scope="col" className="num">Closest points</th>
              </tr>
            </thead>
            <tbody>
              {summary.keyframes.map((k, i) => (
                <tr key={i}>
                  <td>{k.shape_name}</td>
                  <td className="num">{fmt(k.time_sec)} s</td>
                  <td className="num">{k.points}</td>
                  <td className={`num ${k.min_spacing_m < summary.min_distance_m - 1e-6 ? "tone-bad" : ""}`}>
                    {fmt(k.min_spacing_m, 2)} m
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </>
      )}
    </div>
  );
}
