import { spawn } from "node:child_process";
import path from "node:path";
import { fileURLToPath } from "node:url";
import type { PythonBridgeResponse } from "./types.js";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);
const PYTHON_SCRIPT = path.resolve(__dirname, "..", "python", "cli.py");
const PYTHON_BIN = process.env.PYTHON_BIN || "python";

export class PythonBridgeError extends Error {
  code: string;
  detail?: unknown;

  constructor(code: string, message: string, detail?: unknown) {
    super(message);
    this.code = code;
    this.detail = detail;
  }
}

export async function runPythonAction<T>(action: string, payload: unknown): Promise<T> {
  const request = JSON.stringify({ action, payload });

  return await new Promise<T>((resolve, reject) => {
    const proc = spawn(PYTHON_BIN, [PYTHON_SCRIPT], {
      cwd: path.resolve(__dirname, ".."),
      stdio: ["pipe", "pipe", "pipe"],
    });

    let stdout = "";
    let stderr = "";

    proc.stdout.on("data", (chunk) => {
      stdout += chunk.toString();
    });

    proc.stderr.on("data", (chunk) => {
      stderr += chunk.toString();
    });

    proc.on("error", (err) => {
      reject(new PythonBridgeError("PYTHON_SPAWN_ERROR", "Failed to start Python process.", err.message));
    });

    proc.on("close", (code) => {
      if (code !== 0) {
        reject(
          new PythonBridgeError("PYTHON_EXIT_ERROR", "Python process exited with non-zero status.", {
            code,
            stderr: stderr.trim(),
            stdout: stdout.trim(),
          }),
        );
        return;
      }

      let parsed: PythonBridgeResponse<T>;
      try {
        parsed = JSON.parse(stdout);
      } catch (err) {
        reject(
          new PythonBridgeError("PYTHON_PARSE_ERROR", "Could not parse Python JSON response.", {
            stdout: stdout.trim(),
            stderr: stderr.trim(),
            parser: err instanceof Error ? err.message : String(err),
          }),
        );
        return;
      }

      if (!parsed.ok) {
        reject(new PythonBridgeError(parsed.error?.code || "PYTHON_ACTION_ERROR", parsed.error?.message || "Python action failed.", parsed.error?.detail));
        return;
      }

      resolve(parsed.data as T);
    });

    proc.stdin.write(request);
    proc.stdin.end();
  });
}
