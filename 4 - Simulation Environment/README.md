# Simulation Environment - Task D Motion Planning App

This folder now contains a web-based Task D interactive motion-planning application split into:

- `app/` - Vite + React + TypeScript frontend.
- `server/` - Node/Express API that calls Python FK/IK wrappers.

The app supports:

- Waypoint input in **Joint mode** or **Pose mode**.
- **MoveJ** planning with cubic/trapezoid profile.
- **MoveL** planning with Cartesian interpolation and IK at each step.
- 3D robot viewer (hybrid: optional URDF + procedural DH chain).
- Joint trajectory plots (`q1...q6` vs step).
- Status reporting for IK failures, singularity events, and limit issues.
- Built-in sanity checks for Task D acceptance tests.

## 1) Install

From `4 - Simulation Environment/`, install backend and frontend dependencies.

```bash
cd server
npm install

cd ../app
npm install
```

## 2) Run

Start backend first:

```bash
cd "4 - Simulation Environment/server"
npm run dev
```

Then start frontend:

```bash
cd "4 - Simulation Environment/app"
npm run dev
```

Open the Vite URL (default `http://localhost:5173`).

The frontend proxies `/api/*` to backend `http://localhost:8787` via `app/vite.config.ts`.

## 3) Kinematics Integration

Integration method used:

- Frontend calls backend endpoints (`/api/fk`, `/api/fk/batch`, `/api/ik`, `/api/limits`).
- Backend (`server/src/pythonBridge.ts`) spawns Python CLI (`server/python/cli.py`).
- Python wrappers implement FK/IK logic and import your existing DH source:
  - `3 - Software (Core Code)/src/kinematics/dh_parameters.py`

Notes:

- No FK/IK math is implemented in browser JavaScript.
- FK/IK wrappers live in:
  - `server/python/fk.py`
  - `server/python/ik.py`
  - `server/python/se3.py`

## 4) API Summary

- `GET /api/health`
- `GET /api/limits`
- `POST /api/fk` body: `{ "qDeg": [q1..q6] }`
- `POST /api/fk/batch` body: `{ "qDegList": [[...], ...] }`
- `POST /api/ik` body: `{ "T": [[4x4]], "seedDeg": [optional q1..q6] }`

## 5) Acceptance Tests (Sanity Check Panel)

Use the **Sanity Checks** panel in the UI:

1. `FK on [0,-90,90,0,0,0]`
- Computes FK and displays the pose.

2. `IK round-trip on Test 1 pose`
- Runs IK on test-1 FK pose and checks whether one branch matches within wrap-aware tolerance (`<= 0.5 deg/joint`).

3. `MoveJ home -> moderate pose`
- Plans MoveJ from home to `[30,-20,40,10,-30,60]` and starts playback.

4. `MoveL between random reachable poses`
- Uses deterministic seeded random joint samples (within limits), computes start/end reachable poses with FK, plans MoveL, and reports:
  - Straightness metric (max line deviation in mm)
  - IK hard-failure count

## 6) Viewer Behavior

`RobotViewer` uses a hybrid strategy:

- If `VITE_URDF_URL` is provided in `app/.env`, it attempts URDF loading.
- Procedural DH chain rendering is always available and is used as fallback/default.

Example `app/.env`:

```env
VITE_URDF_URL=/assets/robot.urdf
```

## 7) Limitations

- URDF loading depends on valid URDF and mesh paths; missing meshes fall back to procedural chain.
- MoveL uses local midpoint subdivision (`max depth = 3`) for difficult IK transitions; highly constrained targets may still fail.
- Singularity events are surfaced in status (especially wrist singular neighborhoods).
- This implementation relies on local Python + Node runtime and does not currently include production deployment packaging.
