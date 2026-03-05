import { useEffect, useMemo, useState } from "react";
import JointPlot from "./components/JointPlot";
import PlannerControls from "./components/PlannerControls";
import RobotViewer from "./components/RobotViewer";
import SanityCheckPanel from "./components/SanityCheckPanel";
import StatusPanel from "./components/StatusPanel";
import WaypointForm from "./components/WaypointForm";
import { fk, getLimits, ik } from "./kinematics/api";
import { jointDistanceSqDeg, poseToT, TToPose, wrapDeltaDeg } from "./kinematics/conversions";
import { moveJ, moveL } from "./planner";
import { usePlannerState } from "./state/usePlannerState";
import type { Pose, SanityCheckResult, StatusMessage, TrajectoryResult } from "./types";

const HOME_Q = [0, -90, 90, 0, 0, 0];
const MODERATE_Q = [30, -20, 40, 10, -30, 60];

function status(level: "info" | "warning" | "error", message: string): StatusMessage {
  return {
    id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
    level,
    message,
  };
}

function mulberry32(seed: number) {
  return function rng() {
    let t = (seed += 0x6d2b79f5);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function maxLineDeviationMm(points: number[][]): number {
  if (points.length < 3) {
    return 0;
  }

  const p0 = points[0];
  const p1 = points[points.length - 1];
  const v = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
  const vNorm = Math.hypot(v[0], v[1], v[2]) || 1;

  let maxDist = 0;
  for (const p of points.slice(1, -1)) {
    const w = [p[0] - p0[0], p[1] - p0[1], p[2] - p0[2]];
    const cross = [
      w[1] * v[2] - w[2] * v[1],
      w[2] * v[0] - w[0] * v[2],
      w[0] * v[1] - w[1] * v[0],
    ];
    const dist = Math.hypot(cross[0], cross[1], cross[2]) / vNorm;
    if (dist > maxDist) {
      maxDist = dist;
    }
  }
  return maxDist;
}

export default function App() {
  const { state, dispatch } = usePlannerState();
  const [sanityResults, setSanityResults] = useState<SanityCheckResult[]>([]);
  const [runningTestId, setRunningTestId] = useState<string | null>(null);

  const currentPoseText = useMemo(() => {
    const p = state.currentPose.positionMm;
    const rpy = state.currentPose.rpyDeg;
    return `TCP: x=${p[0].toFixed(1)} mm, y=${p[1].toFixed(1)} mm, z=${p[2].toFixed(1)} mm | RPY=${rpy
      .map((v) => v.toFixed(1))
      .join(", ")} deg`;
  }, [state.currentPose]);

  async function refreshFromJointState(qDeg: number[]) {
    const fkResult = await fk(qDeg);
    dispatch({ type: "SET_CURRENT_Q", value: qDeg });
    dispatch({ type: "SET_CURRENT_POSE", value: fkResult.pose });
    dispatch({ type: "SET_CURRENT_LINK_FRAMES", value: fkResult.linkFrames });
    return fkResult;
  }

  function syncCurrentFromTrajectory(traj: TrajectoryResult, index: number) {
    const q = traj.qTraj[index];
    const T = traj.TTraj[index];
    const linkFrames = traj.linkFramesTraj?.[index] || [];

    if (q) {
      dispatch({ type: "SET_CURRENT_Q", value: q });
    }
    if (T) {
      dispatch({ type: "SET_CURRENT_POSE", value: TToPose(T) });
    }
    if (linkFrames.length) {
      dispatch({ type: "SET_CURRENT_LINK_FRAMES", value: linkFrames });
    }
  }

  async function chooseIkSolution(T: number[][], seed: number[]): Promise<number[]> {
    const ikResult = await ik(T, seed);
    if (!ikResult.solutionsDeg.length) {
      throw new Error("IK returned no branch for requested pose.");
    }

    const sorted = [...ikResult.solutionsDeg].sort((a, b) => jointDistanceSqDeg(a, seed) - jointDistanceSqDeg(b, seed));
    return sorted[0];
  }

  useEffect(() => {
    let isMounted = true;

    (async () => {
      try {
        const limits = await getLimits();
        if (!isMounted) {
          return;
        }
        dispatch({ type: "SET_LIMITS", limits });

        const fkHome = await refreshFromJointState([...HOME_Q]);
        if (!isMounted) {
          return;
        }

        dispatch({ type: "SET_START_POSE", value: fkHome.pose });

        const fkGoal = await fk(MODERATE_Q);
        if (!isMounted) {
          return;
        }
        dispatch({ type: "SET_GOAL_POSE", value: fkGoal.pose });
      } catch (err) {
        dispatch({ type: "PUSH_STATUS", value: status("error", `Initialization failed: ${err instanceof Error ? err.message : String(err)}`) });
      }
    })();

    return () => {
      isMounted = false;
    };
  }, [dispatch]);

  useEffect(() => {
    if (!state.isPlaying || !state.trajectory) {
      return;
    }

    const id = window.setInterval(() => {
      const next = state.playIndex + 1;
      const max = state.trajectory?.qTraj.length || 0;
      if (next >= max) {
        dispatch({ type: "SET_IS_PLAYING", value: false });
        return;
      }
      dispatch({ type: "SET_PLAY_INDEX", value: next });
    }, 60);

    return () => window.clearInterval(id);
  }, [state.isPlaying, state.playIndex, state.trajectory, dispatch]);

  useEffect(() => {
    if (!state.trajectory) {
      return;
    }
    syncCurrentFromTrajectory(state.trajectory, state.playIndex);
  }, [state.playIndex, state.trajectory]);

  const handlePlan = async () => {
    dispatch({ type: "SET_IS_PLANNING", value: true });
    dispatch({ type: "SET_IS_PLAYING", value: false });
    dispatch({ type: "CLEAR_STATUSES" });

    try {
      let traj: TrajectoryResult;

      if (state.motionType === "MoveJ") {
        const qStart =
          state.mode === "joint" ? state.startJoint : await chooseIkSolution(poseToT(state.startPose), state.currentQ);
        const qGoal = state.mode === "joint" ? state.goalJoint : await chooseIkSolution(poseToT(state.goalPose), qStart);

        const moveJResult = await moveJ(qStart, qGoal, state.nSteps, state.profile);
        traj = {
          qTraj: moveJResult.qTraj,
          TTraj: moveJResult.TTraj,
          linkFramesTraj: moveJResult.linkFramesTraj,
          failures: moveJResult.failures,
        };
      } else {
        const startT = state.mode === "joint" ? (await fk(state.startJoint)).T : poseToT(state.startPose);
        const goalT = state.mode === "joint" ? (await fk(state.goalJoint)).T : poseToT(state.goalPose);
        traj = await moveL(startT, goalT, state.nSteps);
      }

      dispatch({ type: "SET_TRAJECTORY", value: traj });
      dispatch({ type: "SET_PLAY_INDEX", value: 0 });
      dispatch({ type: "SET_IS_PLAYING", value: false });

      const statuses = traj.failures.map((f) =>
        status(
          f.reason === "IK_FAIL" || f.reason === "LIMIT_BLOCKED" ? "error" : "warning",
          `step ${f.step}: ${f.reason}${f.detail ? ` - ${f.detail}` : ""}`,
        ),
      );
      if (!statuses.length) {
        statuses.push(status("info", `Plan generated with ${traj.qTraj.length} trajectory samples.`));
      }
      dispatch({ type: "SET_STATUSES", value: statuses });

      if (traj.qTraj.length && traj.TTraj.length) {
        syncCurrentFromTrajectory(traj, 0);
      }
    } catch (err) {
      dispatch({ type: "PUSH_STATUS", value: status("error", `Planning failed: ${err instanceof Error ? err.message : String(err)}`) });
    } finally {
      dispatch({ type: "SET_IS_PLANNING", value: false });
    }
  };

  const handlePlay = () => {
    if (!state.trajectory || !state.trajectory.qTraj.length) {
      dispatch({ type: "PUSH_STATUS", value: status("warning", "Plan a trajectory before playback.") });
      return;
    }
    dispatch({ type: "SET_IS_PLAYING", value: true });
  };

  const handlePause = () => dispatch({ type: "SET_IS_PLAYING", value: false });

  const handleReset = async () => {
    dispatch({ type: "SET_IS_PLAYING", value: false });
    dispatch({ type: "SET_PLAY_INDEX", value: 0 });
    dispatch({ type: "SET_TRAJECTORY", value: null });
    dispatch({ type: "CLEAR_STATUSES" });

    try {
      if (state.mode === "joint") {
        await refreshFromJointState([...state.startJoint]);
      } else {
        const q = await chooseIkSolution(poseToT(state.startPose), state.currentQ);
        await refreshFromJointState(q);
      }
    } catch (err) {
      dispatch({ type: "PUSH_STATUS", value: status("error", `Reset failed: ${err instanceof Error ? err.message : String(err)}`) });
    }
  };

  async function runSanityTest(id: string) {
    setRunningTestId(id);

    try {
      if (id === "test1_fk_home") {
        const out = await fk([...HOME_Q]);
        const detail = `Pose: p=[${out.pose.positionMm.map((v) => v.toFixed(2)).join(", ")}] mm, rpy=[${out.pose.rpyDeg
          .map((v) => v.toFixed(2))
          .join(", ")}] deg`;
        setSanityResults((prev) => [
          ...prev.filter((r) => r.id !== id),
          { id, name: "FK home pose", passed: true, detail },
        ]);
        dispatch({ type: "PUSH_STATUS", value: status("info", detail) });
      }

      if (id === "test2_ik_roundtrip") {
        const fkHome = await fk([...HOME_Q]);
        const ikHome = await ik(fkHome.T);
        const matched = ikHome.solutionsDeg.some((q) => q.every((v, idx) => Math.abs(wrapDeltaDeg(v, HOME_Q[idx])) <= 0.5));
        const detail = matched
          ? "At least one IK branch matches home joints within 0.5 deg per joint."
          : "No IK branch matched the expected home configuration within tolerance.";
        setSanityResults((prev) => [
          ...prev.filter((r) => r.id !== id),
          { id, name: "IK round-trip", passed: matched, detail },
        ]);
        dispatch({ type: "PUSH_STATUS", value: status(matched ? "info" : "error", detail) });
      }

      if (id === "test3_movej_demo") {
        const result = await moveJ([...HOME_Q], [...MODERATE_Q], Math.max(50, state.nSteps), state.profile);
        const traj: TrajectoryResult = {
          qTraj: result.qTraj,
          TTraj: result.TTraj,
          linkFramesTraj: result.linkFramesTraj,
          failures: result.failures,
        };
        dispatch({ type: "SET_TRAJECTORY", value: traj });
        dispatch({ type: "SET_PLAY_INDEX", value: 0 });
        dispatch({ type: "SET_IS_PLAYING", value: true });

        const ok = traj.qTraj.length > 0;
        const detail = ok ? `MoveJ planned with ${traj.qTraj.length} points and playback started.` : "MoveJ test failed to produce a trajectory.";
        setSanityResults((prev) => [
          ...prev.filter((r) => r.id !== id),
          { id, name: "MoveJ playback", passed: ok, detail },
        ]);
      }

      if (id === "test4_movel_straight") {
        const rng = mulberry32(7751);
        const randomQ = (): number[] =>
          state.limits.minDeg.map((min, i) => {
            const max = state.limits.maxDeg[i];
            return min + 0.15 * (max - min) + rng() * 0.7 * (max - min);
          });

        const qA = randomQ();
        const qB = randomQ();

        const fkA = await fk(qA);
        const fkB = await fk(qB);
        const traj = await moveL(fkA.T, fkB.T, Math.max(60, state.nSteps));

        const points = traj.TTraj.map((T) => [T[0][3], T[1][3], T[2][3]]);
        const deviation = maxLineDeviationMm(points);
        const hardFailures = traj.failures.filter((f) => f.reason !== "SINGULARITY");

        const passed = hardFailures.length === 0 && deviation <= 5;
        const detail = `max deviation=${deviation.toFixed(3)} mm, hardFailures=${hardFailures.length}`;

        dispatch({ type: "SET_TRAJECTORY", value: traj });
        dispatch({ type: "SET_PLAY_INDEX", value: 0 });

        setSanityResults((prev) => [
          ...prev.filter((r) => r.id !== id),
          { id, name: "MoveL straightness", passed, detail },
        ]);

        dispatch({ type: "PUSH_STATUS", value: status(passed ? "info" : "error", `Sanity test 4: ${detail}`) });
      }
    } catch (err) {
      const message = err instanceof Error ? err.message : String(err);
      setSanityResults((prev) => [
        ...prev.filter((r) => r.id !== id),
        { id, name: id, passed: false, detail: message },
      ]);
      dispatch({ type: "PUSH_STATUS", value: status("error", `Sanity test failed (${id}): ${message}`) });
    } finally {
      setRunningTestId(null);
    }
  }

  return (
    <div className="app-shell">
      <header className="app-header">
        <h1>ME7751 Task D: Interactive Motion Planner</h1>
        <p>
          MoveJ and MoveL planning using backend Python FK/IK wrappers integrated with your existing modified DH parameters.
        </p>
      </header>

      <main className="app-main">
        <div className="left-column">
          <WaypointForm
            mode={state.mode}
            limits={state.limits}
            startJoint={state.startJoint}
            goalJoint={state.goalJoint}
            startPose={state.startPose}
            goalPose={state.goalPose}
            onModeChange={(mode) => dispatch({ type: "SET_MODE", mode })}
            onStartJointChange={(index, value) => {
              const next = [...state.startJoint];
              next[index] = value;
              dispatch({ type: "SET_START_JOINT", value: next });
            }}
            onGoalJointChange={(index, value) => {
              const next = [...state.goalJoint];
              next[index] = value;
              dispatch({ type: "SET_GOAL_JOINT", value: next });
            }}
            onStartPoseChange={(group, index, value) => {
              const next: Pose = {
                ...state.startPose,
                [group]: [...state.startPose[group]] as [number, number, number],
              };
              next[group][index] = value;
              dispatch({ type: "SET_START_POSE", value: next });
            }}
            onGoalPoseChange={(group, index, value) => {
              const next: Pose = {
                ...state.goalPose,
                [group]: [...state.goalPose[group]] as [number, number, number],
              };
              next[group][index] = value;
              dispatch({ type: "SET_GOAL_POSE", value: next });
            }}
            onSetStartToCurrent={() => {
              if (state.mode === "joint") {
                dispatch({ type: "SET_START_JOINT", value: [...state.currentQ] });
              } else {
                dispatch({ type: "SET_START_POSE", value: state.currentPose });
              }
            }}
            onSetGoalToCurrent={() => {
              if (state.mode === "joint") {
                dispatch({ type: "SET_GOAL_JOINT", value: [...state.currentQ] });
              } else {
                dispatch({ type: "SET_GOAL_POSE", value: state.currentPose });
              }
            }}
          />

          <PlannerControls
            motionType={state.motionType}
            profile={state.profile}
            nSteps={state.nSteps}
            isPlanning={state.isPlanning}
            isPlaying={state.isPlaying}
            onMotionTypeChange={(motionType) => dispatch({ type: "SET_MOTION_TYPE", motionType })}
            onProfileChange={(profile) => dispatch({ type: "SET_PROFILE", profile })}
            onStepsChange={(nSteps) => dispatch({ type: "SET_STEPS", nSteps })}
            onPlan={handlePlan}
            onPlay={handlePlay}
            onPause={handlePause}
            onReset={handleReset}
          />

          <SanityCheckPanel runningTestId={runningTestId} results={sanityResults} onRunTest={runSanityTest} />
          <StatusPanel statuses={state.statuses} />
        </div>

        <div className="right-column">
          <RobotViewer linkFrames={state.currentLinkFrames} currentQ={state.currentQ} currentPoseText={currentPoseText} />
          <JointPlot qTraj={state.trajectory?.qTraj || []} currentStep={state.playIndex} limits={state.limits} />
        </div>
      </main>
    </div>
  );
}
