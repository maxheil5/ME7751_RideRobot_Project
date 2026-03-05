# ME7751 Ride Robot Project

A robotics project inspired by KUKA-arm “ride” systems (e.g., Universal-style robotic arm attractions).  
This repo contains (1) kinematic derivations (DH/FK/IK), (2) a simulation environment to run scripted motion “scenarios,” and (3) a scaled physical demo build.

## Goals
- **Course deliverables:** derive and implement FK/IK from a public DH table; verify correctness.
- **Simulation:** render and run repeatable motion scenarios in a clean virtual environment.
- **Physical demo:** 3D-printed, scaled robot arm executing a scripted “ride-like” motion sequence.

## Repository Structure
- `0 - Project Management/` — scope, schedule, tasks, risks, budget
- `1 - Requirements & References/` — prompt, literature, DH sources, datasheets
- `2 - Kinematics (Derivation)/` — DH table, FK/IK derivations, Jacobians
- `3 - Software (Core Code)/` — FK/IK implementation, trajectories, utilities
- `4 - Simulation Environment/` — sim engine, assets, scenarios, renders
- `5 - CAD & 3D Print/` — CAD, STL, drawings, print profiles
- `6 - Electronics & Controls/` — wiring, schematics, control architecture
- `7 - Firmware/` — microcontroller code (if used)
- `8 - Testing & Validation/` — verification, calibration, sim-vs-real comparisons
- `9 - Demos & Media/` — demo scripts, photos, videos, figures
- `10 - Final Submission/` — report + submission-ready artifacts

## Robot Model
The project uses a 6-DOF serial industrial robot model based on **KUKA KR 500 R2830** DH parameters.
- DH reference and citation notes are in `1 - Requirements & References/robot-dh-sources/`.
- Core DH parameter module: `3 - Software (Core Code)/src/kinematics/dh_parameters.py`

## How to Run
### Simulation
Task D app runs as frontend + backend.

1. Backend terminal:
   - `cd "4 - Simulation Environment/server"`
   - `python -m pip install numpy`
   - `npm install`
   - `npm run dev`
2. Frontend terminal:
   - `cd "4 - Simulation Environment/app"`
   - `npm install`
   - `npm run dev`
3. Open the app at:
   - `http://localhost:5173`

Notes:
- Frontend proxy sends `/api/*` requests to backend `http://localhost:8787`.
- If backend is not running, frontend actions will return HTTP/proxy errors.
- Detailed app docs are in `4 - Simulation Environment/README.md`.
- AI Tools were used to help create the app and simulation environment.

### Physical Demo
- CAD + STL: `5 - CAD & 3D Print/`
- Electronics + control: `6 - Electronics & Controls/` and `7 - Firmware/`
- Validation: `8 - Testing & Validation/`
- Physical demo work is extension scope and may be incomplete relative to course Task A-D deliverables.

## Milestones (high-level)
- DH table selected + cited
- FK implemented + verified
- IK implemented + verified
- Simulation scene + scenario 01 running
- Physical prototype (actuated) + scripted demo motion

## License
TBD

## Contact
Max Heil
