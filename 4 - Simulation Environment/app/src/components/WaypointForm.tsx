import type { Limits, Pose, WaypointMode } from "../types";

interface WaypointFormProps {
  mode: WaypointMode;
  limits: Limits;
  startJoint: number[];
  goalJoint: number[];
  startPose: Pose;
  goalPose: Pose;
  onModeChange: (mode: WaypointMode) => void;
  onStartJointChange: (index: number, value: number) => void;
  onGoalJointChange: (index: number, value: number) => void;
  onStartPoseChange: (group: "positionMm" | "rpyDeg", index: number, value: number) => void;
  onGoalPoseChange: (group: "positionMm" | "rpyDeg", index: number, value: number) => void;
  onSetStartToCurrent: () => void;
  onSetGoalToCurrent: () => void;
}

function LabelledNumberInput({
  label,
  value,
  onChange,
  min,
  max,
  unit,
}: {
  label: string;
  value: number;
  onChange: (value: number) => void;
  min?: number;
  max?: number;
  unit: string;
}) {
  const outOfRange = (min !== undefined && value < min) || (max !== undefined && value > max);
  return (
    <label className="num-input">
      <span>
        {label} ({unit})
      </span>
      <input
        type="number"
        value={Number.isFinite(value) ? value : 0}
        min={min}
        max={max}
        step="0.1"
        className={outOfRange ? "input-invalid" : ""}
        onChange={(e) => onChange(Number(e.target.value))}
      />
    </label>
  );
}

function JointSection({
  title,
  joints,
  limits,
  onJointChange,
}: {
  title: string;
  joints: number[];
  limits: Limits;
  onJointChange: (index: number, value: number) => void;
}) {
  return (
    <section className="panel-subsection">
      <h4>{title}</h4>
      <div className="grid-3">
        {joints.map((value, idx) => (
          <LabelledNumberInput
            key={`${title}-q${idx}`}
            label={`q${idx + 1}`}
            unit="deg"
            value={value}
            min={limits.minDeg[idx]}
            max={limits.maxDeg[idx]}
            onChange={(next) => onJointChange(idx, next)}
          />
        ))}
      </div>
    </section>
  );
}

function PoseSection({
  title,
  pose,
  onPoseChange,
}: {
  title: string;
  pose: Pose;
  onPoseChange: (group: "positionMm" | "rpyDeg", index: number, value: number) => void;
}) {
  return (
    <section className="panel-subsection">
      <h4>{title}</h4>
      <div className="grid-3">
        {pose.positionMm.map((value, idx) => (
          <LabelledNumberInput
            key={`${title}-p${idx}`}
            label={["x", "y", "z"][idx]}
            unit="mm"
            value={value}
            onChange={(next) => onPoseChange("positionMm", idx, next)}
          />
        ))}
      </div>
      <div className="grid-3">
        {pose.rpyDeg.map((value, idx) => (
          <LabelledNumberInput
            key={`${title}-rpy${idx}`}
            label={["roll", "pitch", "yaw"][idx]}
            unit="deg"
            value={value}
            onChange={(next) => onPoseChange("rpyDeg", idx, next)}
          />
        ))}
      </div>
    </section>
  );
}

export default function WaypointForm(props: WaypointFormProps) {
  return (
    <section className="panel">
      <div className="panel-header">
        <h3>Waypoints</h3>
        <div className="inline-row">
          <button className={props.mode === "joint" ? "btn-active" : ""} onClick={() => props.onModeChange("joint")}>
            Joint Mode
          </button>
          <button className={props.mode === "pose" ? "btn-active" : ""} onClick={() => props.onModeChange("pose")}>
            Pose Mode
          </button>
        </div>
      </div>

      <div className="inline-row">
        <button onClick={props.onSetStartToCurrent}>Set Start to Current</button>
        <button onClick={props.onSetGoalToCurrent}>Set Goal to Current</button>
      </div>

      {props.mode === "joint" ? (
        <>
          <JointSection title="Start" joints={props.startJoint} limits={props.limits} onJointChange={props.onStartJointChange} />
          <JointSection title="Goal" joints={props.goalJoint} limits={props.limits} onJointChange={props.onGoalJointChange} />
        </>
      ) : (
        <>
          <PoseSection title="Start Pose" pose={props.startPose} onPoseChange={props.onStartPoseChange} />
          <PoseSection title="Goal Pose" pose={props.goalPose} onPoseChange={props.onGoalPoseChange} />
        </>
      )}
    </section>
  );
}
