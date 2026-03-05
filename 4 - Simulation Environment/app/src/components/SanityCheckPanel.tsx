import type { SanityCheckResult } from "../types";

interface SanityCheckPanelProps {
  runningTestId: string | null;
  results: SanityCheckResult[];
  onRunTest: (id: string) => void;
}

const TESTS: { id: string; label: string }[] = [
  { id: "test1_fk_home", label: "1) FK on [0,-90,90,0,0,0]" },
  { id: "test2_ik_roundtrip", label: "2) IK round-trip on Test 1 pose" },
  { id: "test3_movej_demo", label: "3) MoveJ home -> moderate pose" },
  { id: "test4_movel_straight", label: "4) MoveL between random reachable poses" },
];

export default function SanityCheckPanel({ runningTestId, results, onRunTest }: SanityCheckPanelProps) {
  return (
    <section className="panel">
      <div className="panel-header">
        <h3>Sanity Checks</h3>
      </div>
      <div className="stack-sm">
        {TESTS.map((test) => (
          <button key={test.id} disabled={runningTestId !== null} onClick={() => onRunTest(test.id)}>
            {runningTestId === test.id ? "Running..." : test.label}
          </button>
        ))}
      </div>

      <ul className="sanity-list">
        {results.map((result) => (
          <li key={result.id} className={result.passed ? "sanity-pass" : "sanity-fail"}>
            <strong>{result.name}</strong>
            <span>{result.passed ? "PASS" : "FAIL"}</span>
            <p>{result.detail}</p>
          </li>
        ))}
      </ul>
    </section>
  );
}
