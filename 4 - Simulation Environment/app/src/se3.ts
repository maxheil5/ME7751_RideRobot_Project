function matMul3(A: number[][], B: number[][]): number[][] {
  const out = Array.from({ length: 3 }, () => [0, 0, 0]);
  for (let i = 0; i < 3; i += 1) {
    for (let j = 0; j < 3; j += 1) {
      out[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
    }
  }
  return out;
}

function matTranspose3(R: number[][]): number[][] {
  return [
    [R[0][0], R[1][0], R[2][0]],
    [R[0][1], R[1][1], R[2][1]],
    [R[0][2], R[1][2], R[2][2]],
  ];
}

function clamp(value: number, lo = -1, hi = 1): number {
  return Math.max(lo, Math.min(hi, value));
}

export function so3Log(R: number[][]): [number, number, number] {
  const trace = R[0][0] + R[1][1] + R[2][2];
  const theta = Math.acos(clamp((trace - 1) / 2));

  if (theta < 1e-12) {
    return [0, 0, 0];
  }

  if (Math.abs(theta - Math.PI) < 1e-6) {
    const x = Math.sqrt(Math.max((R[0][0] + 1) / 2, 0));
    const y = Math.sqrt(Math.max((R[1][1] + 1) / 2, 0));
    const z = Math.sqrt(Math.max((R[2][2] + 1) / 2, 0));
    const norm = Math.hypot(x, y, z) || 1;
    return [(x / norm) * theta, (y / norm) * theta, (z / norm) * theta];
  }

  const scale = theta / (2 * Math.sin(theta));
  return [
    scale * (R[2][1] - R[1][2]),
    scale * (R[0][2] - R[2][0]),
    scale * (R[1][0] - R[0][1]),
  ];
}

export function so3Exp(w: [number, number, number]): number[][] {
  const theta = Math.hypot(w[0], w[1], w[2]);
  if (theta < 1e-12) {
    return [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1],
    ];
  }

  const kx = w[0] / theta;
  const ky = w[1] / theta;
  const kz = w[2] / theta;

  const K = [
    [0, -kz, ky],
    [kz, 0, -kx],
    [-ky, kx, 0],
  ];

  const K2 = matMul3(K, K);
  const s = Math.sin(theta);
  const c = Math.cos(theta);

  const I = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];

  const out = Array.from({ length: 3 }, () => [0, 0, 0]);
  for (let i = 0; i < 3; i += 1) {
    for (let j = 0; j < 3; j += 1) {
      out[i][j] = I[i][j] + s * K[i][j] + (1 - c) * K2[i][j];
    }
  }
  return out;
}

export function interpolatePose(T0: number[][], T1: number[][], s: number): number[][] {
  const R0 = [
    [T0[0][0], T0[0][1], T0[0][2]],
    [T0[1][0], T0[1][1], T0[1][2]],
    [T0[2][0], T0[2][1], T0[2][2]],
  ];
  const R1 = [
    [T1[0][0], T1[0][1], T1[0][2]],
    [T1[1][0], T1[1][1], T1[1][2]],
    [T1[2][0], T1[2][1], T1[2][2]],
  ];

  const p0 = [T0[0][3], T0[1][3], T0[2][3]];
  const p1 = [T1[0][3], T1[1][3], T1[2][3]];

  const Rdelta = matMul3(matTranspose3(R0), R1);
  const w = so3Log(Rdelta);
  const R = matMul3(R0, so3Exp([w[0] * s, w[1] * s, w[2] * s]));

  const p = [
    p0[0] + s * (p1[0] - p0[0]),
    p0[1] + s * (p1[1] - p0[1]),
    p0[2] + s * (p1[2] - p0[2]),
  ];

  return [
    [R[0][0], R[0][1], R[0][2], p[0]],
    [R[1][0], R[1][1], R[1][2], p[1]],
    [R[2][0], R[2][1], R[2][2], p[2]],
    [0, 0, 0, 1],
  ];
}
