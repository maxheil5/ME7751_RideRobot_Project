## Task C — Step 5: Cross-validation with RoboDK (Pose ↔ IK)

This section cross-validates the analytical IK implementation (MATLAB) against RoboDK for a representative set of target poses. The goal is to verify that:

1. **Pose → RoboDK IK** returns joint configurations consistent with the analytical IK branches (up to angle wrapping).
2. **Analytical IK branches → RoboDK MoveJ/Jog** visually reach the correct target pose.

---

### Selected test poses
We selected 6 poses to cover the main operating regimes:
- **Home (singular wrist case)**: $[0,-90,90,0,0,0]^\circ$
- **Moderate non-singular pose**
- **Moderate pose with different wrist orientation**
- **“Full-branch” pose** (demonstrates many IK branches)
- **Near joint limit pose**
- **Near wrist singular pose** ($q_5 \approx 0$)

Each target pose was imported into RoboDK by pasting the $4\times 4$ homogeneous transform ${}^{0}T_6$ (mm) computed in MATLAB from FK. Targets were stored in the station tree as `Home`, `Pose 2`, …, `Pose 6`.

---

### Consistent frame definitions (critical for matching)
To ensure a fair comparison between MATLAB and RoboDK:
- The **reference frame** in RoboDK was set to the **robot base** frame.
- The **tool frame** was set to the **flange / no TCP offset** (matching the frame-6 definition used in MATLAB FK/IK).
- MATLAB uses the same frame-6 definition (no additional tool transform).

If the TCP or reference frame are not matched, RoboDK and MATLAB will report different poses even when the kinematics are correct.

---

### What RoboDK reports vs. what MATLAB reports
For each target pose, RoboDK’s **“Other configurations”** list reports multiple joint vectors that achieve the same end-effector pose. RoboDK often reports **more than 8** configurations because it includes:

- **Angle wrap equivalents** (e.g., $180^\circ$ and $-180^\circ$, or $270^\circ$ and $-90^\circ$).
- **Wrist singular families** when $q_5 \approx 0$ or $180^\circ$, where $(q_4,q_6)$ become coupled and there are infinitely many equivalent solutions.
- **Multiple solver seeds**, which can yield additional equivalent solutions.

In contrast, the analytical IK returns up to **8 distinct branches** corresponding to:
- 2 shoulder branches × 2 elbow branches × 2 wrist (flip / no-flip) branches  
(when the wrist is non-singular and all branches are within limits).

Therefore, the correct expectation is:
- RoboDK may list more configurations than MATLAB,
- but RoboDK’s returned solutions should match MATLAB’s solutions **up to angle wrapping** and joint limit conventions.

---

### Numerical matching criterion (wrap-aware)
To compare a RoboDK joint solution $q_{\text{RoboDK}}$ to a MATLAB analytical solution $q_{\text{IK}}^{(k)}$, joint differences were computed using a wrap-to-$180^\circ$ operation per joint:

$$
\Delta q_i^{(k)} = \mathrm{wrap}_{180}\!\left(q_{\text{RoboDK},i} - q_{\text{IK},i}^{(k)}\right).
$$

A RoboDK solution is considered consistent with MATLAB if there exists a branch $k$ such that:

- $\max_i |\Delta q_i^{(k)}|$ is small (near numerical tolerance), and
- the corresponding FK check satisfies $FK(q_{\text{IK}}^{(k)}) \approx {}^{0}T_6$ (verified in Task C Steps 2–4).

---

### Visual validation (RoboDK MoveJ/Jog)
For each selected target pose:
1. The robot was commanded (MoveJ / joint jog) to one or more configurations from RoboDK’s solution list.
2. The robot visually reached the target pose (tool frame aligned with the target axes).
3. For representative targets (especially the “full-branch” case), multiple distinct configurations were tested to confirm different IK branches reach the **same** target pose.

Screenshots included in the repository show:
- RoboDK’s `Other configurations` list for each pose (multiple valid IK solutions),
- the robot at the corresponding target pose for each case.

---

### Results summary
- For all selected poses, RoboDK found joint configurations that achieve the same target end-effector pose used in MATLAB.
- MATLAB’s analytical IK solutions were previously verified via $FK(q_{\text{IK}}^{(k)}) \approx T_{\text{test}}$ to numerical precision (Task C Steps 2–4).
- RoboDK’s solution lists often contain more configurations than MATLAB due to wrap equivalences and singularity-related families, but the physically distinct branches align with the analytical branch structure (shoulder/elbow/wrist flip).
- Wrist singular behavior was observed for the Home/near-singular poses, consistent with reduced wrist branching in the analytical formulation.

This cross-validation confirms that the analytical IK implementation is consistent with RoboDK both numerically (via pose agreement) and visually (via simulation alignment).

---

### Files and evidence (what to look at in the repo)
- `taskC_summary.csv` and `taskC_detail.csv`: MATLAB FK→IK→FK verification results.
- RoboDK screenshots:
  - “Other configurations” lists for each pose
  - robot visual confirmation at each target pose
- RoboDK station targets: `Home`, `Pose 2`, `Pose 3`, `Pose 4`, `Pose 5`, `Pose 6`
