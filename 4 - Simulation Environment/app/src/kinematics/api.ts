import type { FkBatchResult, FkResult, IkResult, Limits } from "../types";

const API_BASE = import.meta.env.VITE_API_BASE || "";

interface ApiErrorShape {
  error?: {
    code?: string;
    message?: string;
    detail?: unknown;
  };
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(`${API_BASE}${path}`, {
    headers: {
      "Content-Type": "application/json",
      ...(init?.headers || {}),
    },
    ...init,
  });

  if (!res.ok) {
    let body: ApiErrorShape | undefined;
    try {
      body = (await res.json()) as ApiErrorShape;
    } catch {
      body = undefined;
    }
    const msg = body?.error?.message || `HTTP ${res.status} for ${path}`;
    throw new Error(msg);
  }

  return (await res.json()) as T;
}

export function getLimits(): Promise<Limits> {
  return request<Limits>("/api/limits");
}

export function fk(qDeg: number[]): Promise<FkResult> {
  return request<FkResult>("/api/fk", {
    method: "POST",
    body: JSON.stringify({ qDeg }),
  });
}

export function fkBatch(qDegList: number[][]): Promise<FkBatchResult> {
  return request<FkBatchResult>("/api/fk/batch", {
    method: "POST",
    body: JSON.stringify({ qDegList }),
  });
}

export function ik(T: number[][], seedDeg?: number[]): Promise<IkResult> {
  return request<IkResult>("/api/ik", {
    method: "POST",
    body: JSON.stringify({ T, seedDeg }),
  });
}

export function health() {
  return request<{ status: string; service: string }>("/api/health");
}
