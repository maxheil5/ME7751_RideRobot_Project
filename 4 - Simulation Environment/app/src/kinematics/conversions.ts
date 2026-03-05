import type { Pose } from "../types";

const DEG2RAD = Math.PI / 180;
const RAD2DEG = 180 / Math.PI;

export function degToRad(v: number): number {
  return v * DEG2RAD;
}

export function radToDeg(v: number): number {
  return v * RAD2DEG;
}

export function rpyToR(rollDeg: number, pitchDeg: number, yawDeg: number): number[][] {
  const r = degToRad(rollDeg);
  const p = degToRad(pitchDeg);
  const y = degToRad(yawDeg);

  const cr = Math.cos(r);
  const sr = Math.sin(r);
  const cp = Math.cos(p);
  const sp = Math.sin(p);
  const cy = Math.cos(y);
  const sy = Math.sin(y);

  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

export function RToRpy(R: number[][]): [number, number, number] {
  const sy = Math.max(-1, Math.min(1, -R[2][0]));
  const pitch = Math.asin(sy);

  let roll: number;
  let yaw: number;

  if (Math.abs(Math.cos(pitch)) > 1e-8) {
    roll = Math.atan2(R[2][1], R[2][2]);
    yaw = Math.atan2(R[1][0], R[0][0]);
  } else {
    yaw = 0;
    roll = Math.atan2(-R[0][1], R[1][1]);
  }

  return [radToDeg(roll), radToDeg(pitch), radToDeg(yaw)];
}

export function poseToT(pose: Pose): number[][] {
  const R = rpyToR(pose.rpyDeg[0], pose.rpyDeg[1], pose.rpyDeg[2]);
  return [
    [R[0][0], R[0][1], R[0][2], pose.positionMm[0]],
    [R[1][0], R[1][1], R[1][2], pose.positionMm[1]],
    [R[2][0], R[2][1], R[2][2], pose.positionMm[2]],
    [0, 0, 0, 1],
  ];
}

export function TToPose(T: number[][]): Pose {
  const R = [
    [T[0][0], T[0][1], T[0][2]],
    [T[1][0], T[1][1], T[1][2]],
    [T[2][0], T[2][1], T[2][2]],
  ];
  const rpyDeg = RToRpy(R);
  return {
    positionMm: [T[0][3], T[1][3], T[2][3]],
    rpyDeg,
  };
}

export function wrapDeltaDeg(a: number, b: number): number {
  let d = a - b;
  d = ((d + 180) % 360 + 360) % 360 - 180;
  return d;
}

export function jointDistanceSqDeg(a: number[], b: number[]): number {
  let sum = 0;
  for (let i = 0; i < 6; i += 1) {
    const d = wrapDeltaDeg(a[i], b[i]);
    sum += d * d;
  }
  return sum;
}
