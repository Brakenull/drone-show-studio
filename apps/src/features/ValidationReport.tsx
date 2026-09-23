// Result of `studio_bridge validate` (docs/5-studio_gui.md §6.1).

import type { Validation } from "../bridge/types";

const fmt = (v: number, d = 1) => v.toFixed(d);

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
            <div>
              <dt>Holding area</dt>
              <dd>
                {summary.holding_area.size[0]} × {summary.holding_area.size[1]} m, {summary.holding_area.layers}{" "}
                {summary.holding_area.layers === 1 ? "layer" : "layers"}
              </dd>
            </div>
            <div>
              <dt>Launch grid</dt>
              <dd>{summary.holding_area.grid_spacing_m} m</dd>
            </div>
          </dl>
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
