import { useReducer } from "react";
import type {
  Limits,
  MotionType,
  MoveJProfile,
  Pose,
  StatusMessage,
  TrajectoryResult,
  WaypointMode,
} from "../types";

export interface PlannerState {
  limits: Limits;
  mode: WaypointMode;
  motionType: MotionType;
  profile: MoveJProfile;
  nSteps: number;
  startJoint: number[];
  goalJoint: number[];
  startPose: Pose;
  goalPose: Pose;
  currentQ: number[];
  currentPose: Pose;
  currentLinkFrames: number[][][];
  trajectory: TrajectoryResult | null;
  playIndex: number;
  isPlaying: boolean;
  isPlanning: boolean;
  statuses: StatusMessage[];
}

const HOME_Q = [0, -90, 90, 0, 0, 0];

const ZERO_POSE: Pose = {
  positionMm: [0, 0, 0],
  rpyDeg: [0, 0, 0],
};

const DEFAULT_LIMITS: Limits = {
  minDeg: [-185, -130, -100, -350, -120, -350],
  maxDeg: [185, 20, 144, 350, 120, 350],
};

export const initialPlannerState: PlannerState = {
  limits: DEFAULT_LIMITS,
  mode: "joint",
  motionType: "MoveJ",
  profile: "cubic",
  nSteps: 100,
  startJoint: [...HOME_Q],
  goalJoint: [30, -20, 40, 10, -30, 60],
  startPose: ZERO_POSE,
  goalPose: ZERO_POSE,
  currentQ: [...HOME_Q],
  currentPose: ZERO_POSE,
  currentLinkFrames: [],
  trajectory: null,
  playIndex: 0,
  isPlaying: false,
  isPlanning: false,
  statuses: [],
};

type Action =
  | { type: "SET_LIMITS"; limits: Limits }
  | { type: "SET_MODE"; mode: WaypointMode }
  | { type: "SET_MOTION_TYPE"; motionType: MotionType }
  | { type: "SET_PROFILE"; profile: MoveJProfile }
  | { type: "SET_STEPS"; nSteps: number }
  | { type: "SET_START_JOINT"; value: number[] }
  | { type: "SET_GOAL_JOINT"; value: number[] }
  | { type: "SET_START_POSE"; value: Pose }
  | { type: "SET_GOAL_POSE"; value: Pose }
  | { type: "SET_CURRENT_Q"; value: number[] }
  | { type: "SET_CURRENT_POSE"; value: Pose }
  | { type: "SET_CURRENT_LINK_FRAMES"; value: number[][][] }
  | { type: "SET_TRAJECTORY"; value: TrajectoryResult | null }
  | { type: "SET_PLAY_INDEX"; value: number }
  | { type: "SET_IS_PLAYING"; value: boolean }
  | { type: "SET_IS_PLANNING"; value: boolean }
  | { type: "SET_STATUSES"; value: StatusMessage[] }
  | { type: "PUSH_STATUS"; value: StatusMessage }
  | { type: "CLEAR_STATUSES" }
  | { type: "RESET_PLAYBACK" };

function reducer(state: PlannerState, action: Action): PlannerState {
  switch (action.type) {
    case "SET_LIMITS":
      return { ...state, limits: action.limits };
    case "SET_MODE":
      return { ...state, mode: action.mode };
    case "SET_MOTION_TYPE":
      return { ...state, motionType: action.motionType };
    case "SET_PROFILE":
      return { ...state, profile: action.profile };
    case "SET_STEPS":
      return { ...state, nSteps: action.nSteps };
    case "SET_START_JOINT":
      return { ...state, startJoint: action.value };
    case "SET_GOAL_JOINT":
      return { ...state, goalJoint: action.value };
    case "SET_START_POSE":
      return { ...state, startPose: action.value };
    case "SET_GOAL_POSE":
      return { ...state, goalPose: action.value };
    case "SET_CURRENT_Q":
      return { ...state, currentQ: action.value };
    case "SET_CURRENT_POSE":
      return { ...state, currentPose: action.value };
    case "SET_CURRENT_LINK_FRAMES":
      return { ...state, currentLinkFrames: action.value };
    case "SET_TRAJECTORY":
      return { ...state, trajectory: action.value };
    case "SET_PLAY_INDEX":
      return { ...state, playIndex: action.value };
    case "SET_IS_PLAYING":
      return { ...state, isPlaying: action.value };
    case "SET_IS_PLANNING":
      return { ...state, isPlanning: action.value };
    case "SET_STATUSES":
      return { ...state, statuses: action.value };
    case "PUSH_STATUS":
      return { ...state, statuses: [...state.statuses, action.value] };
    case "CLEAR_STATUSES":
      return { ...state, statuses: [] };
    case "RESET_PLAYBACK":
      return { ...state, playIndex: 0, isPlaying: false };
    default:
      return state;
  }
}

export function usePlannerState() {
  const [state, dispatch] = useReducer(reducer, initialPlannerState);
  return { state, dispatch };
}
