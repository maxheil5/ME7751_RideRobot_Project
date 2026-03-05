import type { StatusMessage } from "../types";

interface StatusPanelProps {
  statuses: StatusMessage[];
}

export default function StatusPanel({ statuses }: StatusPanelProps) {
  return (
    <section className="panel">
      <div className="panel-header">
        <h3>Status</h3>
      </div>

      {!statuses.length ? <p className="muted">No warnings or errors.</p> : null}

      <ul className="status-list">
        {statuses.map((status) => (
          <li key={status.id} className={`status-item status-${status.level}`}>
            <span className="status-level">{status.level.toUpperCase()}</span>
            <span>{status.message}</span>
          </li>
        ))}
      </ul>
    </section>
  );
}
