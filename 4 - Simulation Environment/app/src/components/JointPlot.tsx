import { Line, LineChart, ReferenceLine, ResponsiveContainer, Tooltip, XAxis, YAxis } from "recharts";
import type { Limits } from "../types";

interface JointPlotProps {
  qTraj: number[][];
  currentStep: number;
  limits: Limits;
}

export default function JointPlot({ qTraj, currentStep, limits }: JointPlotProps) {
  const data = qTraj.map((q, step) => ({
    step,
    q1: q[0],
    q2: q[1],
    q3: q[2],
    q4: q[3],
    q5: q[4],
    q6: q[5],
  }));

  if (!data.length) {
    return (
      <section className="panel">
        <div className="panel-header">
          <h3>Joint Trajectory</h3>
        </div>
        <p className="muted">Plan a trajectory to view q1..q6 plots.</p>
      </section>
    );
  }

  return (
    <section className="panel">
      <div className="panel-header">
        <h3>Joint Trajectory (deg vs step)</h3>
      </div>
      <div className="joint-plot-grid">
        {[1, 2, 3, 4, 5, 6].map((jointIdx) => {
          const key = `q${jointIdx}` as keyof (typeof data)[number];
          const min = limits.minDeg[jointIdx - 1];
          const max = limits.maxDeg[jointIdx - 1];
          const hasLimitBreach = data.some((d) => (d[key] as number) < min || (d[key] as number) > max);

          return (
            <div key={key} className="joint-plot-item">
              <h5>
                {key}
                {hasLimitBreach ? " (limit warning)" : ""}
              </h5>
              <ResponsiveContainer width="100%" height={150}>
                <LineChart data={data} margin={{ top: 8, right: 10, left: 0, bottom: 4 }}>
                  <XAxis dataKey="step" tick={{ fontSize: 11 }} />
                  <YAxis tick={{ fontSize: 11 }} width={36} />
                  <Tooltip />
                  <ReferenceLine x={currentStep} stroke="#f59e0b" strokeDasharray="5 5" />
                  <ReferenceLine y={min} stroke="#dc2626" strokeDasharray="4 3" />
                  <ReferenceLine y={max} stroke="#dc2626" strokeDasharray="4 3" />
                  <Line
                    type="monotone"
                    dataKey={key}
                    stroke={hasLimitBreach ? "#dc2626" : "#2563eb"}
                    strokeWidth={2}
                    dot={false}
                    isAnimationActive={false}
                  />
                </LineChart>
              </ResponsiveContainer>
            </div>
          );
        })}
      </div>
    </section>
  );
}
