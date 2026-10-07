// Suggestions for the moments a rain window doesn't cover: each
// option with its numbers and how much of the uncovered time it closes on its own, ranked by that. The app
// never applies one by itself; each leads to an explicit action.

import type { ReactNode } from "react";
import type { Suggestion, Suggestions } from "../bridge/types";
import { formatTime } from "../replay/sampling";

const seconds = (v: number) => `${Math.round(v)} s`;
const plural = (n: number, one: string, many: string) => (n === 1 ? one : many);

interface Props {
  result: Suggestions;
  /** False once the window, rule or return paths changed since `result` was computed. */
  current: boolean;
  disabled: boolean;
  /** The row whose button started the running planning job, and that job's progress to show in it. */
  busyIndex: number | null;
  progress: ReactNode;
  /** Called with a row's index when its button starts planning. */
  onStart: (index: number) => void;
  onPlanPoints: (points: [number, number][]) => void;
  onPlanReturn: (formation: number) => void;
  onSetAlert: (alertMmH: number) => void;
}

export function SuggestionList({ result, current, disabled, busyIndex, progress, onStart, onPlanPoints, onPlanReturn, onSetAlert }: Props) {
  if (result.items.length === 0) {
    return <p className="muted small">Nothing to suggest: every moment is covered with a {seconds(result.window_sec)} window.</p>;
  }
  return (
    <ol className={`suggestions ${current ? "" : "suggestions-stale"}`} aria-label="Suggestions, most useful first">
      {result.items.map((item, i) => (
        <li key={i} className="suggestion">
          <Closes item={item} total={result.uncovered_sec} />
          <div className="suggestion-body">
            <p className="suggestion-title">
              {title(item)}
              {item.estimate && (
                <span className="suggestion-tag" title="Until Stage 2 plans it and the safety check passes it, this is an estimate.">
                  estimate
                </span>
              )}
            </p>
            <p className="muted small">{detail(item)}</p>
          </div>
          <div className="suggestion-action">
            {busyIndex === i ? (
              <span className="muted small">Planning…</span>
            ) : (
              <Action
                item={item}
                disabled={disabled || !current}
                onPlanPoints={(points) => {
                  onStart(i);
                  onPlanPoints(points);
                }}
                onPlanReturn={(k) => {
                  onStart(i);
                  onPlanReturn(k);
                }}
                onSetAlert={onSetAlert}
              />
            )}
          </div>
          {busyIndex === i && progress && <div className="suggestion-progress">{progress}</div>}
        </li>
      ))}
    </ol>
  );
}

/** How much of the uncovered time this option closes, as a bar against all of it. */
function Closes({ item, total }: { item: Suggestion; total: number }) {
  const share = total > 0 ? Math.min(1, item.closes_sec / total) : 0;
  const label = item.closes_all
    ? `Closes all ${seconds(total)}`
    : item.closes_sec > 0.05
      ? `Closes ${seconds(item.closes_sec)} of ${seconds(total)}`
      : "Closes nothing";
  return (
    <div className="suggestion-closes" title={`${label} of uncovered show time`}>
      <span className={`suggestion-meter ${item.closes_all ? "is-all" : ""}`} aria-hidden="true">
        <span style={{ width: `${share * 100}%` }} />
      </span>
      <span className="small num">{label}</span>
    </div>
  );
}

function title(item: Suggestion): string {
  const n = item.numbers;
  const name = item.formation_name ?? "";
  switch (item.kind) {
    case "abort_points": {
      const count = n.points?.length ?? 0;
      return `Plan ${count} ${plural(count, "return", "returns")} from inside the move to ${name}`;
    }
    case "earlier_trigger":
      return `Start the return ${seconds(n.lead_sec ?? 0)} before the rain alert`;
    case "plan_return":
      return `Plan a return path from ${name}`;
    case "faster_return":
      return n.possible
        ? `Plan the return from ${name} again at its minimum time`
        : `The return from ${name} is already as fast as the motion limits allow`;
    case "design":
      return n.return_leg
        ? `Shorten the show's return leg from ${name}`
        : `Bring ${name} closer to the holding area`;
  }
}

function detail(item: Suggestion): string {
  const n = item.numbers;
  const name = item.formation_name ?? "";
  switch (item.kind) {
    case "abort_points": {
      const pts = n.points ?? [];
      const at = pts.map((p) => `${formatTime(p.time_sec)} (${seconds(p.duration_sec)} home)`).join(", ");
      return `From ${at}, the fleet turns for home instead of finishing the move. ${pts.length === 1 ? "It is" : "Each is"} planned like a show transition.`;
    }
    case "earlier_trigger":
      return n.alert_mm_h != null
        ? `With the rain rising steadily, an alert level of ${n.alert_mm_h} mm/h (now ${n.current_alert_mm_h} mm/h) is reached that much earlier.`
        : "A lower alert level can't give that much time; the return would have to start on a rain forecast or radar warning.";
    case "plan_return":
      return `About ${seconds(n.duration_sec ?? 0)} home from ${name}, instead of flying the rest of the show to its return leg.`;
    case "faster_return":
      if (!n.possible) {
        return n.farthest_drone != null && n.farthest_m != null
          ? `It takes ${seconds(n.return_sec ?? 0)}: drone ${n.farthest_drone} has ${Math.round(n.farthest_m)} m to its slot${
              n.farthest_area != null ? ` in holding area ${n.farthest_area + 1}` : ""
            }.`
          : `It takes ${seconds(n.return_sec ?? 0)}.`;
      }
      return n.new_sec != null
        ? `It takes ${seconds(n.return_sec ?? 0)}${n.target_sec != null ? ` (planned to a ${seconds(n.target_sec)} target)` : n.attempts && n.attempts > 1 ? ` (lengthened by ${n.attempts - 1} safety-check ${plural(n.attempts - 1, "retry", "retries")})` : ""}; at its minimum time, about ${seconds(n.new_sec)}.`
        : `It was planned to a ${seconds(n.target_sec ?? 0)} target; planning it at its minimum time shows how much shorter it can be.`;
    case "design": {
      const what = n.return_leg ? "The return leg" : "Its return";
      const need = n.return_needed_sec ?? 0;
      const fix =
        need > 0
          ? `it would need ${seconds(need)} (${seconds(n.short_sec ?? 0)} shorter)`
          : `even an instant return would leave the ${seconds(n.move_sec ?? 0)} move into it too long`;
      const home = n.farthest_area != null ? `holding area ${n.farthest_area + 1} (where drone ${n.farthest_drone}, the farthest, lands)` : "the holding area";
      return `${what} takes ${seconds(n.return_sec ?? 0)}; ${fix}. Change the design in Blender: move ${name} closer to ${home}, lower it, or put it earlier in the show.`;
    }
  }
}

function Action({
  item,
  disabled,
  onPlanPoints,
  onPlanReturn,
  onSetAlert,
}: Pick<Props, "disabled" | "onPlanPoints" | "onPlanReturn" | "onSetAlert"> & { item: Suggestion }) {
  const a = item.action;
  if (!a) return null;
  if (a.kind === "plan_points") {
    return (
      <button onClick={() => onPlanPoints(a.points)} disabled={disabled}>
        Plan {plural(a.points.length, "it", "them")}
      </button>
    );
  }
  if (a.kind === "set_alert") {
    return (
      <button onClick={() => onSetAlert(a.alert_mm_h)} disabled={disabled}>
        Use {a.alert_mm_h} mm/h
      </button>
    );
  }
  return (
    <button onClick={() => onPlanReturn(a.formation)} disabled={disabled}>
      {a.kind === "plan_return" ? "Plan it" : "Plan again"}
    </button>
  );
}
