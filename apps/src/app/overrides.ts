// Planner-settings overrides (docs/5-studio_gui.md §6.2). Values are keyed by dotted path; only settings
// that differ from the run's baseline are sent. The warning rules mirror
// tools/studio_bridge/config_fields.py safety_warnings(), which records the same warnings in run.json.

import type { ConfigField, ConfigWarning, Overrides } from "../bridge/types";

export type Value = number | boolean | string;
export type Values = Record<string, Value>;

export function flatten(tree: Overrides, prefix = ""): Values {
  const out: Values = {};
  for (const [key, value] of Object.entries(tree)) {
    // drone_core accepts "weights" as another name for "solver_weights".
    const path = `${prefix}${prefix === "" && key === "weights" ? "solver_weights" : key}`;
    if (typeof value === "object" && value !== null) Object.assign(out, flatten(value, `${path}.`));
    else out[path] = value;
  }
  return out;
}

export function nest(values: Values): Overrides {
  const out: Overrides = {};
  for (const [path, value] of Object.entries(values)) {
    const keys = path.split(".");
    let node = out;
    for (const key of keys.slice(0, -1)) node = (node[key] ??= {}) as Overrides;
    node[keys[keys.length - 1]] = value;
  }
  return out;
}

/** Only real changes: an override equal to the baseline is dropped. */
export function changed(fields: ConfigField[], values: Values): Values {
  const out: Values = {};
  for (const f of fields) {
    if (f.path in values && values[f.path] !== f.baseline) out[f.path] = values[f.path];
  }
  return out;
}

export function warningsFor(fields: ConfigField[], values: Values): ConfigWarning[] {
  const effective = (path: string) => {
    const f = fields.find((x) => x.path === path);
    return path in values ? values[path] : f?.baseline;
  };
  const out: ConfigWarning[] = [];
  for (const f of fields) {
    if (!f.risky || !(f.path in values)) continue;
    const before = f.baseline;
    const after = values[f.path];
    if (before === after) continue;
    const worse =
      f.risky === "lower"
        ? Number(after) < Number(before)
        : f.risky === "higher"
          ? Number(after) > Number(before)
          : before === true && after === false;
    if (!worse) continue;
    const verb = f.risky === "lower" ? "lowered" : f.risky === "higher" ? "raised" : null;
    out.push({
      path: f.path,
      label: f.label,
      message: verb ? `${f.label} ${verb} from ${before} to ${after}.` : `${f.label} turned off.`,
    });
  }
  // The check only finds pairs whose planning-distance boxes touch (2-phase_2.md §5, known limit).
  const floorPath = "solver.continuous_gatekeeper.min_allowable_distance_m";
  const floor = Number(effective(floorPath));
  const enforced =
    Number(effective("safety.min_distance_m")) * (1 + Number(effective("solver.collision_margin_fraction")));
  if (floor > 2 * enforced) {
    out.push({
      path: floorPath,
      label: "Required distance",
      message: `Required distance ${floor} m is more than twice the planning distance (${enforced.toFixed(2)} m). The check can't see pairs that far apart, so it may pass a show that breaks it. Raise the planning distance too.`,
    });
  }
  return out;
}

/** Parse what was typed into a field; an error string when it isn't a valid value. */
export function parse(f: ConfigField, text: string): Value | { error: string } {
  if (f.kind === "boolean" || f.kind === "choice") return text === "true" ? true : text === "false" ? false : text;
  const v = Number(text);
  if (text.trim() === "" || !Number.isFinite(v)) return { error: "Enter a number." };
  if (f.kind === "integer" && !Number.isInteger(v)) return { error: "Enter a whole number." };
  if (f.min !== null && v < f.min) return { error: `At least ${f.min}.` };
  return v;
}
