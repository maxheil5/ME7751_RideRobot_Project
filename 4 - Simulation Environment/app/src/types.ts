export type WaypointMode = "joint" | "pose";
export type MotionType = "MoveJ" | "MoveL";
export type MoveJProfile = "cubic" | "trapezoid";

export type PlannerFailureReason = "IK_FAIL" | "SINGULARITY" | "LIMIT_BLOCKED";

export interface PlannerFailure {
  step: number;
  reason: PlannerFailureReason;
  detail?: string;
}

export interface Pose {
  positionMm: [number, number, number];
  rpyDeg: [number, number, number];
}

export interface Limits {
  minDeg: number[];
  maxDeg: number[];
}

export interface IkMeta {
  branch: [number, number, number];
  singular: boolean;
  reason: string;
}

export interface FkResult {
  T: number[][];
  pose: Pose;
  linkFrames: number[][][];
  warnings: string[];
}

export interface FkBatchResult {
  TList: number[][][];
  poseList: Pose[];
  linkFramesList: number[][][][];
  warnings: string[];
}

export interface IkResult {
  solutionsDeg: number[][];
  meta: IkMeta[];
  warnings: string[];
}

export interface TrajectoryResult {
  qTraj: number[][];
  TTraj: number[][][];
  failures: PlannerFailure[];
  linkFramesTraj?: number[][][][];
}

export interface StatusMessage {
  id: string;
  level: "info" | "warning" | "error";
  message: string;
}

export interface SanityCheckResult {
  id: string;
  name: string;
  passed: boolean;
  detail: string;
}
