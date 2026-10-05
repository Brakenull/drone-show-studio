// Verdict card: the headline of a result, tinted by its status.

/** A verdict card: a tinted panel with a glowing status light, its headline, and what follows. */
export function VerdictCard({
  tone,
  title,
  inner,
  children,
}: {
  tone: "ok" | "bad" | "warn" | "idle";
  title: React.ReactNode;
  inner?: boolean;
  children?: React.ReactNode;
}) {
  return (
    <section className={`verdict-card verdict-card-${tone} ${inner ? "verdict-card-inner" : ""}`}>
      <div className="verdict">
        <span className="verdict-icon" aria-hidden="true">
          <span className={`light light-${tone}`} />
        </span>
        <h2>{title}</h2>
      </div>
      {children}
    </section>
  );
}
