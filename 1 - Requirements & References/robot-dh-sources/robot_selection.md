# Robot Selection

## Selected Robot (TBD)
- Robot name/model:
- Rationale (1–3 bullets):
  - Public DH table available (citable)
  - 6-DOF serial architecture compatible with analytical IK (spherical wrist preferred)
  - Plausible “ride arm” analog (inspiration: KUKA RoboCoaster-style motion)

## Candidate Shortlist
1) KUKA KR6 / KR series (public DH tables exist in papers; easier to scale physically)
2) KUKA KR500-class (ride-relevant; DH info may be modified DH or partial)
3) UR5 / UR10 style (tons of public kinematics; not KUKA-themed but very documented)
4) PUMA 560 (classic analytical IK; lots of public derivations)

## Decision Criteria
- DH table quality (complete numeric values + convention stated)
- IK tractability (closed-form branches)
- Fit for physical prototype scale (reasonable link lengths)
- Availability of meshes/URDF for clean simulation
