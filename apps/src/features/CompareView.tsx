// Compare two runs' Stage 2 results: closest-pair distance over time
// overlaid, the headline numbers side by side, and the planner settings that differ.

import { useEffect, useMemo, useState } from "react";
import { Alert, Select, Table } from "antd";
import { readRunJson, runJob } from "../bridge/api";
import type { ConfigField, Overrides, RunRecord } from "../bridge/types";
import { duration, runCreated, runName, STATUS } from "../app/format";
import { flatten, type Value } from "../app/overrides";
import { SERIES_COLORS, SeparationCompare } from "../replay/SeparationCompare";
import { formatTime, metres } from "../replay/sampling";
import type { ReplayHeader, Separation } from "../replay/types";

interface Loaded {
  separation: Separation | null;
  floor: number | null;
  settings: Map<string, Value>; // every field's effective value
  fields: ConfigField[];
}

/** A row of the side-by-side table: a measure and its value in each run. */
interface Measure {
  label: string;
  cells: { text: string; className?: string }[];
}

const hasResult = (r: RunRecord) => r.stage2.status === "succeeded" || r.stage2.status === "failed_safety";

async function load(run: RunRecord): Promise<Loaded> {
  const [header, separation, overrides, config] = await Promise.all([
    readRunJson<ReplayHeader>(run.run_id, "stage2/replay/replay.json"),
    readRunJson<Separation>(run.run_id, "stage2/replay/separation.json"),
    readRunJson<Overrides>(run.run_id, "stage2/config_overrides.json"),
    runJob(["config", run.run_dir]),
  ]);
  const event = config.events.find((e) => e.type === "config");
  const fields = event && event.type === "config" ? event.fields : [];
  const values = flatten(overrides ?? {});
  const settings = new Map(fields.map((f) => [f.path, f.path in values ? values[f.path] : f.baseline] as const));
  return { separation, floor: header?.overlays.gatekeeper_floor_m ?? null, settings, fields };
}

/** The run to offer first: the one this was copied from, a copy of this one, then the same show file. */
function suggest(run: RunRecord, candidates: RunRecord[]): RunRecord | undefined {
  return (
    candidates.find((r) => r.run_id === run.copied_from) ??
    candidates.find((r) => r.copied_from === run.run_id) ??
    candidates.find((r) => r.input.sha256 === run.input.sha256) ??
    candidates[0]
  );
}

const show = (v: Value | undefined) =>
  v === undefined ? "n/a" : typeof v === "boolean" ? (v ? "On" : "Off") : v === "disabled" ? "Off" : String(v);

export function CompareView({ run, runs }: { run: RunRecord; runs: RunRecord[] }) {
  const candidates = useMemo(() => runs.filter((r) => r.run_id !== run.run_id && hasResult(r)), [runs, run.run_id]);
  const [otherId, setOtherId] = useState<string | null>(() => suggest(run, candidates)?.run_id ?? null);
  const other = candidates.find((r) => r.run_id === otherId) ?? null;
  const [data, setData] = useState<{ a: Loaded; b: Loaded } | null>(null);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    if (!other || !hasResult(run)) return;
    let live = true;
    setData(null);
    setError(null);
    Promise.all([load(run), load(other)])
      .then(([a, b]) => live && setData({ a, b }))
      .catch((e) => live && setError(String(e)));
    return () => {
      live = false;
    };
  }, [run, other]);

  if (!hasResult(run)) {
    return (
      <div className="page">
        <header className="page-head">
          <h1>Compare</h1>
        </header>
        <p>Run Stage 2 on this run first. Comparing needs a result from both runs.</p>
      </div>
    );
  }

  const sameShow = other && other.input.sha256 === run.input.sha256;
  const nameA = "This run";
  const nameB = other ? runName(other) : "";
  const differences =
    data && other
      ? data.a.fields.filter((f) => show(data.a.settings.get(f.path)) !== show(data.b.settings.get(f.path)))
      : [];

  return (
    <div className="page">
      <header className="page-head">
        <h1>Compare</h1>
        <p className="lede">
          How close the drones come over the whole show in this run and another one, for example the same show
          planned with different settings.
        </p>
      </header>

      {candidates.length === 0 ? (
        <p>
          No other run has a Stage 2 result yet. On the Stage 2 tab, "Try these settings in a new run" makes a copy
          of this show to plan with other settings.
        </p>
      ) : (
        <>
          <label className="field compare-pick">
            <span className="field-label">Compare with</span>
            <Select
              style={{ width: "min(36rem, 100%)" }}
              value={otherId ?? undefined}
              onChange={setOtherId}
              options={candidates.map((r) => ({
                value: r.run_id,
                label: `${runName(r)}, ${runCreated(r)} (${STATUS[r.stage2.status].label.toLowerCase()}${r.input.sha256 === run.input.sha256 ? ", same show" : ""})`,
              }))}
            />
          </label>
          {other && !sameShow && (
            <Alert
              type="warning"
              showIcon
              title="These runs plan different show files, so their formations and timing differ too, not only the settings."
            />
          )}
          {error && <Alert type="error" showIcon title={error} />}
          {!data && !error && <p className="status-line">Loading both runs…</p>}

          {data && other && (
            <>
              {data.a.separation && data.b.separation ? (
                <SeparationCompare
                  a={{ label: nameA, separation: data.a.separation, floor: data.a.floor }}
                  b={{ label: nameB, separation: data.b.separation, floor: data.b.floor }}
                />
              ) : (
                <Alert
                  type="info"
                  showIcon
                  title={`${data.a.separation ? nameB : nameA} has no replay data yet. Build it on that run's Replay tab.`}
                />
              )}

              <Table<Measure>
                className="compare-table"
                size="small"
                pagination={false}
                rowKey="label"
                dataSource={[
                  {
                    label: "Stage 2",
                    cells: [run, other].map((r) => ({
                      text: STATUS[r.stage2.status].label,
                      className: `tone-${STATUS[r.stage2.status].tone}`,
                    })),
                  },
                  {
                    label: "Closest approach (sampled)",
                    cells: [data.a, data.b].map((d) => {
                      const w = d.separation?.worst;
                      const below = w && d.floor !== null && (w.distance_m ?? Infinity) < d.floor;
                      return {
                        text: w ? `${metres(w.distance_m, 2)} at ${formatTime(w.time_sec)}` : "n/a",
                        className: below ? "tone-bad" : undefined,
                      };
                    }),
                  },
                  { label: "Required distance", cells: [data.a, data.b].map((d) => ({ text: metres(d.floor, 2) })) },
                  {
                    label: "Show length",
                    cells: [data.a, data.b].map((d) => {
                      const s = d.separation;
                      return { text: s ? duration(s.times[s.times.length - 1] - s.times[0]) : "n/a" };
                    }),
                  },
                  { label: "Planning time", cells: [run, other].map((r) => ({ text: duration(r.stage2.wall_time_sec) })) },
                  ...(run.stage2.status === "failed_safety" || other.stage2.status === "failed_safety"
                    ? [
                        {
                          label: "Rejected at",
                          cells: [run, other].map((r) => ({
                            text: r.stage2.transition
                              ? `${r.stage2.transition.from_keyframe.replace("holding_area", "holding area")} to ${r.stage2.transition.to_keyframe.replace("holding_area", "holding area")}`
                              : "n/a",
                          })),
                        },
                      ]
                    : []),
                ]}
                columns={[
                  {
                    title: <span className="visually-hidden">Measure</span>,
                    dataIndex: "label",
                    className: "measure",
                    onCell: () => ({ scope: "row" }),
                  },
                  ...[nameA, nameB].map((n, i) => ({
                    title: (
                      <>
                        <span className="series-swatch" style={{ background: SERIES_COLORS[i] }} aria-hidden="true" />
                        {n}
                      </>
                    ),
                    key: String(i),
                    onCell: (m: Measure) => ({ className: m.cells[i].className }),
                    render: (_: unknown, m: Measure) => m.cells[i].text,
                  })),
                ]}
              />

              <h3>Planner settings that differ</h3>
              {differences.length === 0 ? (
                <p className="muted">Both runs used the same planner settings.</p>
              ) : (
                <Table<ConfigField>
                  className="compare-table"
                  size="small"
                  pagination={false}
                  rowKey="path"
                  dataSource={differences}
                  columns={[
                    {
                      title: "Setting",
                      key: "setting",
                      className: "measure",
                      render: (_, f) => (
                        <>
                          {f.label} <span className="setting-help">{f.group}</span>
                        </>
                      ),
                    },
                    { title: nameA, key: "a", render: (_, f) => show(data.a.settings.get(f.path)) },
                    { title: nameB, key: "b", render: (_, f) => show(data.b.settings.get(f.path)) },
                  ]}
                />
              )}
            </>
          )}
        </>
      )}
    </div>
  );
}
