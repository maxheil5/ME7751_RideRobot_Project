import { fkBatch, getLimits, ik } from "./kinematics/api";
import { jointDistanceSqDeg } from "./kinematics/conversions";
import { interpolatePose } from "./se3";
import type { MoveJProfile, PlannerFailure, TrajectoryResult } from "./types";

const MAX_SUBDIVISION_DEPTH = 3;
const SINGULARITY_SIN_THRESHOLD = 1e-4;

interface SolveSuccess {
  ok: true;
  q: number[];
  singular: boolean;
  singularReason: string;
}

interface SolveFailure {
  ok: false;
  failure: PlannerFailure;
}

type SolveOutcome = SolveSuccess | SolveFailure;

interface SegmentOutcome {
  points: SolveSuccess[];
  failure?: PlannerFailure;
}

function clampStepCount(nSteps: number): number {
  return Math.max(2, Math.floor(nSteps));
}

function cubicProfile(t: number): number {
  return 3 * t * t - 2 * t * t * t;
}

function trapezoidProfile(t: number, accelFrac = 0.2): number {
  const a = Math.max(0.05, Math.min(0.45, accelFrac));
  const A = 1 / (a * (1 - a));
  const v = A * a;

  if (t < a) {
    return 0.5 * A * t * t;
  }
  if (t <= 1 - a) {
    return 0.5 * A * a * a + v * (t - a);
  }

  const td = 1 - t;
  return 1 - 0.5 * A * td * td;
}

function scalarProfile(profile: MoveJProfile, t: number): number {
  if (profile === "trapezoid") {
    return trapezoidProfile(t);
  }
  return cubicProfile(t);
}

function withinLimits(q: number[], minDeg: number[], maxDeg: number[]): boolean {
  for (let i = 0; i < 6; i += 1) {
    if (q[i] < minDeg[i] - 1e-9 || q[i] > maxDeg[i] + 1e-9) {
      return false;
    }
  }
  return true;
}

async function solvePoseWithIk(T: number[][], seed: number[] | undefined, step: number, minDeg: number[], maxDeg: number[]): Promise<SolveOutcome> {
  try {
    const ikResult = await ik(T, seed);
    if (!ikResult.solutionsDeg.length) {
      const isLimit = ikResult.warnings.some((w) => w.toLowerCase().includes("limit"));
      return {
        ok: false,
        failure: {
          step,
          reason: isLimit ? "LIMIT_BLOCKED" : "IK_FAIL",
          detail: ikResult.warnings.join(" ") || "No IK branch returned for target pose.",
        },
      };
    }

    const candidateIndices = ikResult.solutionsDeg
      .map((q, idx) => ({ q, idx }))
      .filter(({ q }) => withinLimits(q, minDeg, maxDeg));

    if (!candidateIndices.length) {
      return {
        ok: false,
        failure: {
          step,
          reason: "LIMIT_BLOCKED",
          detail: "IK produced branches, but none satisfied joint limits.",
        },
      };
    }

    const ordered = seed
      ? candidateIndices.sort((a, b) => jointDistanceSqDeg(a.q, seed) - jointDistanceSqDeg(b.q, seed))
      : candidateIndices;

    const chosen = ordered[0];
    const meta = ikResult.meta[chosen.idx];
    const q5Rad = (chosen.q[4] * Math.PI) / 180;
    const singularViaSin = Math.abs(Math.sin(q5Rad)) < SINGULARITY_SIN_THRESHOLD;

    return {
      ok: true,
      q: chosen.q,
      singular: Boolean(meta?.singular) || singularViaSin,
      singularReason: meta?.reason || "wrist singularity neighborhood",
    };
  } catch (err) {
    return {
      ok: false,
      failure: {
        step,
        reason: "IK_FAIL",
        detail: err instanceof Error ? err.message : String(err),
      },
    };
  }
}

async function resolveSegment(
  TStart: number[][],
  TEnd: number[][],
  qStart: number[],
  depth: number,
  step: number,
  minDeg: number[],
  maxDeg: number[],
): Promise<SegmentOutcome> {
  const direct = await solvePoseWithIk(TEnd, qStart, step, minDeg, maxDeg);
  if (direct.ok) {
    return { points: [direct] };
  }

  if (depth >= MAX_SUBDIVISION_DEPTH) {
    return {
      points: [],
      failure: {
        ...direct.failure,
        detail: `${direct.failure.detail || "IK failed."} Subdivision limit reached.`,
      },
    };
  }

  const TMid = interpolatePose(TStart, TEnd, 0.5);

  const firstHalf = await resolveSegment(TStart, TMid, qStart, depth + 1, step, minDeg, maxDeg);
  if (firstHalf.failure) {
    return firstHalf;
  }

  const qMid = firstHalf.points[firstHalf.points.length - 1]?.q || qStart;
  const secondHalf = await resolveSegment(TMid, TEnd, qMid, depth + 1, step, minDeg, maxDeg);
  if (secondHalf.failure) {
    return secondHalf;
  }

  return {
    points: [...firstHalf.points, ...secondHalf.points],
  };
}

export async function moveJ(
  q0: number[],
  q1: number[],
  nSteps: number,
  profile: MoveJProfile,
): Promise<{ qTraj: number[][]; TTraj: number[][][]; linkFramesTraj?: number[][][][]; failures: PlannerFailure[] }> {
  const steps = clampStepCount(nSteps);
  const qTraj: number[][] = [];

  for (let i = 0; i < steps; i += 1) {
    const t = i / (steps - 1);
    const s = scalarProfile(profile, t);
    qTraj.push(q0.map((q, idx) => q + s * (q1[idx] - q)));
  }

  const fkData = await fkBatch(qTraj);
  return {
    qTraj,
    TTraj: fkData.TList,
    linkFramesTraj: fkData.linkFramesList,
    failures: fkData.warnings.map((message, idx) => ({
      step: idx,
      reason: "LIMIT_BLOCKED" as const,
      detail: message,
    })),
  };
}

export async function moveL(T0: number[][], T1: number[][], nSteps: number): Promise<TrajectoryResult> {
  const steps = clampStepCount(nSteps);
  const failures: PlannerFailure[] = [];

  const limits = await getLimits();
  const minDeg = limits.minDeg;
  const maxDeg = limits.maxDeg;

  const startSolve = await solvePoseWithIk(T0, undefined, 0, minDeg, maxDeg);
  if (!startSolve.ok) {
    return {
      qTraj: [],
      TTraj: [],
      failures: [startSolve.failure],
    };
  }

  const qTraj: number[][] = [startSolve.q];
  let prevQ = startSolve.q;
  let prevTarget = T0;

  if (startSolve.singular) {
    failures.push({
      step: 0,
      reason: "SINGULARITY",
      detail: `Start waypoint near singular wrist: ${startSolve.singularReason}`,
    });
  }

  for (let i = 1; i < steps; i += 1) {
    const s = i / (steps - 1);
    const target = interpolatePose(T0, T1, s);

    const segment = await resolveSegment(prevTarget, target, prevQ, 0, i, minDeg, maxDeg);
    if (segment.failure) {
      failures.push(segment.failure);
      break;
    }

    for (const solvedPoint of segment.points) {
      qTraj.push(solvedPoint.q);
      prevQ = solvedPoint.q;
      if (solvedPoint.singular) {
        failures.push({
          step: i,
          reason: "SINGULARITY",
          detail: `MoveL step solved with singularity handling: ${solvedPoint.singularReason}`,
        });
      }
    }

    prevTarget = target;
  }

  if (!qTraj.length) {
    return {
      qTraj,
      TTraj: [],
      failures,
    };
  }

  const fkData = await fkBatch(qTraj);
  return {
    qTraj,
    TTraj: fkData.TList,
    linkFramesTraj: fkData.linkFramesList,
    failures: failures.concat(
      fkData.warnings.map((message, idx) => ({
        step: idx,
        reason: "LIMIT_BLOCKED" as const,
        detail: message,
      })),
    ),
  };
}
