# Project Prompt Notes (ME7751 Project 1)

## Objective
Implement a complete kinematics + motion planning workflow for a 6-DOF serial robot:
- FK derivation + implementation
- Analytical IK derivation + implementation
- Thorough verification
- Motion planning app (MoveJ + MoveL)
- Short paper + presentation
- (Optional) GitHub deployment

## Deliverables
- [ ] Robot model defined + DH table documented
- [ ] FK derivation + code
- [ ] IK derivation + code (all branches where applicable)
- [ ] Verification results (tables/plots + reference comparison)
- [ ] Motion planning app demo (MoveJ + MoveL + visualization)
- [ ] Short paper (≤4 pages, double column)
- [ ] Slide deck + demo plan
- [ ] AI usage documentation (screenshots/notes)

## Constraints / Notes
- Robot must be 6-DOF serial (no external track axis in the core math).
- Use a consistent DH convention across derivation + code + app.
- Verification must be quantitative (tolerances stated).
- Keep artifacts ready for submission packaging.

## Open Questions (to resolve early)
- Exact robot choice for DH table + justification:
- DH convention: standard DH vs modified DH?
- Simulation stack: PyBullet vs MuJoCo vs RoboDK-only?
- App stack: Streamlit vs PyQt vs web app?
