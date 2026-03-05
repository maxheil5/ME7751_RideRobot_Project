import cors from "cors";
import express from "express";
import { PythonBridgeError, runPythonAction } from "./pythonBridge.js";
import type { ApiErrorPayload, FkBatchResponse, FkResponse, IkResponse, Mat4 } from "./types.js";

const app = express();
const PORT = Number(process.env.PORT || 8787);

app.use(cors());
app.use(express.json({ limit: "2mb" }));

function sendError(res: express.Response<ApiErrorPayload>, status: number, code: string, message: string, detail?: unknown) {
  res.status(status).json({
    error: {
      code,
      message,
      detail,
    },
  });
}

function isJointVector(value: unknown): value is number[] {
  return Array.isArray(value) && value.length === 6 && value.every((v) => Number.isFinite(v));
}

function isMat4(value: unknown): value is Mat4 {
  return (
    Array.isArray(value) &&
    value.length === 4 &&
    value.every((row) => Array.isArray(row) && row.length === 4 && row.every((v) => Number.isFinite(v)))
  );
}

app.get("/api/health", (_req, res) => {
  res.json({ status: "ok", service: "me7751-taskd-server" });
});

app.get("/api/limits", async (_req, res) => {
  try {
    const limits = await runPythonAction<{ minDeg: number[]; maxDeg: number[] }>("limits", {});
    res.json(limits);
  } catch (err) {
    if (err instanceof PythonBridgeError) {
      sendError(res, 500, err.code, err.message, err.detail);
      return;
    }
    sendError(res, 500, "INTERNAL_ERROR", "Unexpected server error.");
  }
});

app.post("/api/fk", async (req, res) => {
  const qDeg = req.body?.qDeg;
  if (!isJointVector(qDeg)) {
    sendError(res, 400, "BAD_REQUEST", "qDeg must be an array of 6 finite numbers.");
    return;
  }

  try {
    const fk = await runPythonAction<FkResponse>("fk", { qDeg });
    res.json(fk);
  } catch (err) {
    if (err instanceof PythonBridgeError) {
      sendError(res, 500, err.code, err.message, err.detail);
      return;
    }
    sendError(res, 500, "INTERNAL_ERROR", "Unexpected server error.");
  }
});

app.post("/api/fk/batch", async (req, res) => {
  const qDegList = req.body?.qDegList;
  if (!Array.isArray(qDegList) || qDegList.some((q) => !isJointVector(q))) {
    sendError(res, 400, "BAD_REQUEST", "qDegList must be an array of 6-element joint arrays.");
    return;
  }

  try {
    const fkBatch = await runPythonAction<FkBatchResponse>("fk_batch", { qDegList });
    res.json(fkBatch);
  } catch (err) {
    if (err instanceof PythonBridgeError) {
      sendError(res, 500, err.code, err.message, err.detail);
      return;
    }
    sendError(res, 500, "INTERNAL_ERROR", "Unexpected server error.");
  }
});

app.post("/api/ik", async (req, res) => {
  const T = req.body?.T;
  const seedDeg = req.body?.seedDeg;

  if (!isMat4(T)) {
    sendError(res, 400, "BAD_REQUEST", "T must be a 4x4 homogeneous transform matrix.");
    return;
  }

  if (seedDeg !== undefined && !isJointVector(seedDeg)) {
    sendError(res, 400, "BAD_REQUEST", "seedDeg must be a 6-element array when provided.");
    return;
  }

  try {
    const ik = await runPythonAction<IkResponse>("ik", { T, seedDeg });
    res.json(ik);
  } catch (err) {
    if (err instanceof PythonBridgeError) {
      sendError(res, 500, err.code, err.message, err.detail);
      return;
    }
    sendError(res, 500, "INTERNAL_ERROR", "Unexpected server error.");
  }
});

app.use((_req, res) => {
  sendError(res, 404, "NOT_FOUND", "API route not found.");
});

app.listen(PORT, () => {
  // Chosen integration: web frontend -> Node API -> Python FK/IK wrappers.
  console.log(`Task D backend listening on http://localhost:${PORT}`);
});
