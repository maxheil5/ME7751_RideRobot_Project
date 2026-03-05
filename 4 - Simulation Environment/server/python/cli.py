from __future__ import annotations

import json
import sys
import traceback
from typing import Any

from fk import fk_batch, fk_kinematics, get_joint_limits
from ik import ik_solve


def _ok(data: Any) -> dict[str, Any]:
    return {"ok": True, "data": data}


def _error(code: str, message: str, detail: Any = None) -> dict[str, Any]:
    return {
        "ok": False,
        "error": {
            "code": code,
            "message": message,
            "detail": detail,
        },
    }


def handle_request(request: dict[str, Any]) -> dict[str, Any]:
    action = request.get("action")
    payload = request.get("payload", {})

    if action == "limits":
        return _ok(get_joint_limits())

    if action == "fk":
        q_deg = payload.get("qDeg")
        if not isinstance(q_deg, list) or len(q_deg) != 6:
            return _error("BAD_REQUEST", "payload.qDeg must be a 6-element list.")
        return _ok(fk_kinematics(q_deg))

    if action == "fk_batch":
        q_deg_list = payload.get("qDegList")
        if not isinstance(q_deg_list, list):
            return _error("BAD_REQUEST", "payload.qDegList must be a list of joint vectors.")
        for q in q_deg_list:
            if not isinstance(q, list) or len(q) != 6:
                return _error("BAD_REQUEST", "Every qDegList entry must be a 6-element list.")
        return _ok(fk_batch(q_deg_list))

    if action == "ik":
        T = payload.get("T")
        seed_deg = payload.get("seedDeg")
        if seed_deg is not None and (not isinstance(seed_deg, list) or len(seed_deg) != 6):
            return _error("BAD_REQUEST", "seedDeg must be a 6-element list when provided.")
        return _ok(ik_solve(T, seed_deg=seed_deg))

    return _error("UNKNOWN_ACTION", f"Unsupported action '{action}'.")


def main() -> None:
    try:
        raw = sys.stdin.read()
        if not raw.strip():
            print(json.dumps(_error("BAD_REQUEST", "No JSON request payload received on stdin.")))
            return

        request = json.loads(raw)
        response = handle_request(request)
        print(json.dumps(response))
    except Exception as exc:  # noqa: BLE001
        print(
            json.dumps(
                _error(
                    "PYTHON_RUNTIME_ERROR",
                    "Unhandled Python error while processing request.",
                    {
                        "exception": str(exc),
                        "traceback": traceback.format_exc(),
                    },
                )
            )
        )


if __name__ == "__main__":
    main()
