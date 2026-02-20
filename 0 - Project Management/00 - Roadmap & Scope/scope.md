# Scope

## Project Title
ME7751 Ride Robot Project — Kinematics, Motion Planning App, and Scaled Physical Demo

## One-sentence Summary
Derive and implement FK/IK for a 6-DOF serial robot with a public DH table, verify it rigorously against a reference model, build an interactive motion planning app (MoveJ/MoveL + 3D visualization), and extend the work into a scaled fixed-base physical demo executing a scripted “ride-like” motion sequence.

---

## Primary Objectives (Course Deliverables)
1. **Robot selection + parameters**
   - Select a **6-DOF serial manipulator** (industrial spherical wrist or UR-style).
   - Obtain a **public DH table** (and/or extract DH from RoboDK) and document joint limits/home pose.

2. **Task A — Forward Kinematics (FK)**
   - Assign DH frames and derive all link transforms.
   - Implement FK in code.
   - Validate FK numerically vs reference (RoboDK or equivalent).

3. **Task B — Analytical Inverse Kinematics (IK)**
   - Develop closed-form IK (all branches).
   - Implement IK with singularity and joint-limit handling.

4. **Task C — Thorough IK Verification**
   - Round-trip verification: FK(q) → IK(T) → FK(q_k) ≈ T for all branches.
   - Cross-check select cases against RoboDK.

5. **Task D — Interactive Motion Planning Application**
   - GUI that supports:
     - Two waypoints (joint or Cartesian).
     - **MoveJ** (joint interpolation + FK along path)
     - **MoveL** (Cartesian interpolation + IK along path)
     - Joint-angle plots and 3D animation.

6. **Task F — Short Paper**
   - Conference-style paper (≤4 pages, double-column) summarizing FK, IK, verification, and the app.

7. **Task G — Presentation**
   - 10–15 min presentation with demo and key results.

8. **Task E — GitHub Deployment (Optional / Bonus)**
   - Public (or instructor-shared) repo with README, documentation, and optionally a deployed app URL.

---

## Extension Objectives (Non-required, but planned)
These are explicitly **secondary** and must not jeopardize completion of the course deliverables.

A. **Simulation “Ride Scene” Environment**
- A clean rendered environment (virtual scene) that:
  - Loads the robot model (mesh/URDF or procedural geometry).
  - Runs scripted motion scenarios (predefined sequences).
  - Produces renders/video clips for demos/paper/presentation.

B. **Scaled Physical Demo (Fixed Base)**
- Design + fabricate a scaled version of the robot (3D printed structure).
- Select actuators + control electronics.
- Implement a repeatable scenario (sequence of end-effector poses or joint targets).
- Demonstrate motion that resembles a “ride arm” sequence (without track motion).

---

## In Scope
### Technical
- DH parameter documentation (standard or modified DH — must be consistent everywhere).
- FK derivation and implementation.
- Analytical IK derivation (multi-branch) and implementation.
- Verification harness (unit tests + tables + tolerances).
- Motion planning:
  - Interpolation (trapezoidal velocity profile or polynomial smoothing).
  - MoveJ and MoveL logic.
- Visualization:
  - 3D animation of the robot.
  - Joint trajectory plots.

### Documentation
- Short paper + references.
- README + “how to run” instructions.
- AI tool usage documentation (what was used, where, and what was modified).

### Physical Demo (Extension)
- CAD design for printed links/joints.
- Assembly documentation, wiring diagram, BOM.
- Control loop implementation (high-level joint control).
- Scripted scenario playback.

---

## Out of Scope (to prevent scope creep)
- Track-mounted motion (external 7th axis) and multi-robot coordination.
- High-fidelity structural FEA / fatigue life.
- Passenger safety systems (restraints, safety-rated controllers, redundancy).
- Real-time collision avoidance and full motion planning in cluttered environments (beyond basic interpolation).
- Closed-loop vision-based control (unless time permits and clearly optional).

---

## Key Constraints
- Robot must be **6-DOF serial** for the kinematics derivation.
- DH convention must be **consistent** across:
  - derivation,
  - code,
  - simulation,
  - app.
- Verification must demonstrate numerical agreement to within chosen tolerances.
- Physical demo is **fixed base** and scaled (actuator torque limits will drive design).

---

## Assumptions
- A credible DH table exists publicly or can be extracted from RoboDK for the chosen robot.
- The robot has a structure compatible with analytical IK (e.g., spherical wrist) OR a documented analytical method exists for the chosen structure.
- Simulation stack can load robot geometry (URDF/GLTF/OBJ) and visualize trajectories.
- For the physical demo, the chosen scale keeps joint torques within actuator capability.

---

## Success Criteria (Acceptance Summary)
Minimum successful submission:
- FK matches reference poses for multiple test configurations within tolerance.
- IK returns all valid branches; each branch round-trips (FK(IK(T)) ≈ T).
- GUI supports MoveJ + MoveL, plots joint trajectories, and animates motion.
- Paper + slides delivered with clear verification results and screenshots.

Stretch success:
- Simulation environment produces clean renders of scenarios.
- Physical robot executes a repeatable scripted demo sequence.
- GitHub repo is polished and (optionally) app is deployed.
