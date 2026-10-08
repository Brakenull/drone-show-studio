// What a running or finished job shows: its progress, and its output log.

import { Collapse, Progress, Spin } from "antd";

/** How far a job is: a solid bar once the share done is known, a spinner while it is starting. */
export function JobProgress({ pct, title }: { pct: number | null; title?: string }) {
  return (
    <div className="job-progress" title={title}>
      {pct === null ? (
        <Spin size="small" />
      ) : (
        <Progress
          percent={pct}
          showInfo={false}
          size="small"
          strokeColor="var(--primary)"
          railColor="var(--border)"
        />
      )}
    </div>
  );
}

/** A folded log: `title` on the header, `text` in a scrolling monospace box. */
export function LogPane({ title, text }: { title: string; text: string }) {
  return (
    <Collapse
      ghost
      size="small"
      className="log"
      styles={{ header: { paddingInline: 0 }, body: { paddingInline: 0 } }}
      items={[{ key: "log", label: <span className="muted">{title}</span>, children: <pre>{text}</pre> }]}
    />
  );
}
