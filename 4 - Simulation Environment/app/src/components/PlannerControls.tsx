import type { MotionType, MoveJProfile } from "../types";

interface PlannerControlsProps {
  motionType: MotionType;
  profile: MoveJProfile;
  nSteps: number;
  isPlanning: boolean;
  isPlaying: boolean;
  onMotionTypeChange: (motionType: MotionType) => void;
  onProfileChange: (profile: MoveJProfile) => void;
  onStepsChange: (steps: number) => void;
  onPlan: () => void;
  onPlay: () => void;
  onPause: () => void;
  onReset: () => void;
}

export default function PlannerControls(props: PlannerControlsProps) {
  return (
    <section className="panel">
      <div className="panel-header">
        <h3>Planner Controls</h3>
      </div>

      <div className="grid-2">
        <label className="num-input">
          <span>Motion Type</span>
          <select value={props.motionType} onChange={(e) => props.onMotionTypeChange(e.target.value as MotionType)}>
            <option value="MoveJ">MoveJ</option>
            <option value="MoveL">MoveL</option>
          </select>
        </label>

        <label className="num-input">
          <span>MoveJ Profile</span>
          <select value={props.profile} onChange={(e) => props.onProfileChange(e.target.value as MoveJProfile)}>
            <option value="cubic">cubic</option>
            <option value="trapezoid">trapezoid</option>
          </select>
        </label>
      </div>

      <label className="num-input">
        <span>Steps ({props.nSteps})</span>
        <input
          type="range"
          min={50}
          max={200}
          value={props.nSteps}
          onChange={(e) => props.onStepsChange(Number(e.target.value))}
        />
      </label>

      <div className="inline-row">
        <button disabled={props.isPlanning} onClick={props.onPlan}>
          {props.isPlanning ? "Planning..." : "Plan"}
        </button>
        <button disabled={props.isPlanning || props.isPlaying} onClick={props.onPlay}>
          Play
        </button>
        <button disabled={!props.isPlaying} onClick={props.onPause}>
          Pause
        </button>
        <button onClick={props.onReset}>Reset</button>
      </div>
    </section>
  );
}
