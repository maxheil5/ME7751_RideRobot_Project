export type Mat4 = number[][];

export interface ApiErrorPayload {
  error: {
    code: string;
    message: string;
    detail?: unknown;
  };
}

export interface PythonBridgeResponse<T> {
  ok: boolean;
  data?: T;
  error?: {
    code: string;
    message: string;
    detail?: unknown;
  };
}

export interface FkResponse {
  T: Mat4;
  pose: {
    positionMm: [number, number, number];
    rpyDeg: [number, number, number];
  };
  linkFrames: Mat4[];
  warnings: string[];
}

export interface FkBatchResponse {
  TList: Mat4[];
  poseList: {
    positionMm: [number, number, number];
    rpyDeg: [number, number, number];
  }[];
  linkFramesList: Mat4[][];
  warnings: string[];
}

export interface IkMeta {
  branch: [number, number, number];
  singular: boolean;
  reason: string;
}

export interface IkResponse {
  solutionsDeg: number[][];
  meta: IkMeta[];
  warnings: string[];
}
