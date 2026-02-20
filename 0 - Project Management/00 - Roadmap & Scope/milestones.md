# Milestones & Roadmap

## Roadmap Philosophy
Deliver course-graded items first (FK/IK/Verification/App/Paper/Presentation). The simulation polish and physical demo are scheduled as stretch work after the core math + app are stable.

---

## Phase 0 — Setup & Robot Definition
**M0.1 Repo + folder structure complete**
- Repo created, baseline structure committed, README started.

**M0.2 Robot selected + DH source locked**
- Robot choice documented.
- DH parameters acquired (public citation and/or RoboDK extraction).
- Joint limits + home configuration documented.

**Exit Criteria**
- `2 - Kinematics (Derivation)/dh-table/` contains the chosen DH table (and citation).
- `1 - Requirements & References/robot-dh-sources/` contains source notes.

---

## Phase 1 — FK (Task A)
**M1.1 FK derivation complete**
- Individual transforms derived and checked symbolically.

**M1.2 FK implementation + reference validation**
- FK code implemented.
- Numerical comparison vs reference for multiple q test vectors.

**Exit Criteria**
- FK matches reference pose to tolerance for N test cases.
- Report-ready table/screenshots generated.

---

## Phase 2 — IK (Task B)
**M2.1 IK structure + branch plan finalized**
- Identify decoupling approach (e.g., spherical wrist).
- Enumerate solution branches (target up to 8 where applicable).

**M2.2 IK implementation complete**
- Handles joint limits and singularities gracefully.
- Returns all feasible solutions (filtered/flagged).

**Exit Criteria**
- IK returns consistent branch solutions for general targets.
- Singular cases handled (documented behavior).

---

## Phase 3 — Verification (Task C)
**M3.1 Round-trip test harness**
- Generate q_test set (include near-singular and boundary conditions).
- Automate FK→IK→FK checks.

**M3.2 Cross-validation vs RoboDK**
- Select representative poses for RoboDK comparison.
- Record screenshots / numeric comparisons.

**Exit Criteria**
- Verification results summarized in a clear table with tolerances.
- Any failure modes explained (and either fixed or bounded).

---

## Phase 4 — Motion Planning App (Task D)
**M4.1 App scaffolding + robot viewer**
- 3D robot rendered.
- Joint sliders and pose display working.

**M4.2 MoveJ**
- Joint interpolation + smoothing.
- 3D playback and joint plots.

**M4.3 MoveL**
- Cartesian interpolation (pos + orientation via log/exp).
- IK at each step + playback and plots.

**Exit Criteria**
- User can define start/goal (joint or Cartesian).
- MoveJ and MoveL both produce smooth motion and plots.

---

## Phase 5 — Documentation (Tasks F & G, and optional E)
**M5.1 Paper draft**
- FK/IK/verification/app sections populated.
- Figures and tables inserted.

**M5.2 Presentation**
- Slides built; demo clip or live demo prepared.

**M5.3 GitHub repo polish (Optional)**
- README “How to run”
- Clean install steps
- Optional deployment link

**Exit Criteria**
- Paper ≤4 pages and polished.
- Presentation ready with demo.

---

## Phase 6 — Extensions (Stretch)
### Simulation “Ride Scene”
**M6.1 Clean rendered environment**
- Scene + lighting + camera rigs.
- Exported clips/images.

**M6.2 Scripted scenario playback**
- Scenario file format defined (e.g., JSON: keyframes/poses).
- Playback produces stable repeats.

### Physical Demo
**M6.3 CAD + print + assembly**
- Joint designs validated for strength and assembly.
- Printed prototype assembled.

**M6.4 Electronics + control**
- Actuators selected, powered, controlled.
- Basic homing/calibration.

**M6.5 Scripted demo sequence**
- A fixed scenario runs end-to-end repeatably.

---

## Milestone-to-Deliverable Mapping (Quick)
- Task A: M1.*
- Task B: M2.*
- Task C: M3.*
- Task D: M4.*
- Task F/G: M5.*
- Optional GitHub/Deployment: M5.3
- Simulation/Physical demo: M6.*
