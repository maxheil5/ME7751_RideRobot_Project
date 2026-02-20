# Definition of Done (DoD)

This document defines “done” at three levels:
1) feature-level done (FK/IK/etc.),
2) deliverable-level done (paper/app),
3) project-level done (submission readiness).

---

## Global DoD Rules (Applies to Everything)
- Reproducible: a fresh clone can run/build with documented steps.
- Consistent notation: DH convention and frame naming consistent across derivation, code, and app.
- Verified: numerical checks exist (not just “looks right”).
- Documented: assumptions, limitations, and AI tool usage documented.

---

## FK DoD (Task A)
A feature is “done” when:
- DH table is finalized and referenced.
- All 6 link transforms are derived and implemented.
- FK returns a full SE(3) pose (R, p) for the tool frame.
- Validation:
  - Tested on multiple joint configurations.
  - Pose error metrics defined and reported (e.g., position norm error + rotation error angle).
  - Matches reference within stated tolerance.

Artifacts:
- FK derivation notes (PDF/MD).
- FK code + unit tests.
- Validation table/screenshots.

---

## IK DoD (Task B)
A feature is “done” when:
- IK approach is clearly stated (decoupling, geometry, etc.).
- Returns all solution branches (or explicitly documents why fewer are possible).
- Handles:
  - wrist singularity / elbow singularity behaviors,
  - unreachable targets,
  - joint limit checking (filter or flag).

Artifacts:
- IK derivation notes (PDF/MD).
- IK code + tests.
- Branch enumeration documented.

---

## Verification DoD (Task C)
“Done” means:
- A systematic test suite exists:
  - q_test set includes nominal, near-singular, and limit cases.
  - For each q_test: T = FK(q_test)
  - IK(T) returns {q_k}
  - For each q_k: FK(q_k) ≈ T within tolerance.
- Results summarized in a clear table.
- Selected tests cross-checked in RoboDK (or equivalent) with screenshots/numbers.

---

## Motion Planning App DoD (Task D)
“Done” means the GUI supports:
- Waypoints: start and goal as joint configs OR Cartesian pose.
- MoveJ:
  - Smooth joint interpolation
  - Joint-angle plots
  - 3D animation playback
- MoveL:
  - Linear position interpolation
  - Orientation interpolation via log/exp (or equivalent)
  - IK at each step
  - Joint-angle plots
  - 3D animation playback
- Usability:
  - clear labels, reset buttons, play/pause, and status messages for IK failures.

Artifacts:
- App source code
- Screenshots / demo clips
- Run instructions

---

## Paper DoD (Task F)
“Done” means:
- ≤4 pages, double-column format
- Includes:
  - robot identification + DH table
  - FK summary + validation results
  - IK summary + branch/singularity discussion
  - verification results
  - app architecture + screenshots + sample trajectories
  - references
- Figures readable and cited.

---

## Presentation DoD (Task G)
“Done” means:
- 10–15 min slide deck
- Includes live or recorded demo
- Covers robot structure, FK/IK results, verification, and app behavior.

---

## Extension DoD (Stretch)
### Simulation
- A “clean room” scene exists (lighting/camera presets).
- Scenario scripts are stored as files and replay deterministically.
- Renders exported to `9 - Demos & Media/`.

### Physical Demo
- Robot moves through a scripted scenario repeatably.
- Mechanical, electrical, and code configuration documented.
- Clear safety constraints stated (power limits, emergency stop procedure, etc.).

---

## Submission Readiness DoD
- All required deliverables present in `10 - Final Submission/`.
- README includes “how to run” (app + verification).
- No broken links, missing figures, or undocumented setup steps.
