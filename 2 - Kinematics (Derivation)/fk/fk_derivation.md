# Task A — Forward Kinematics (FK)
**Robot:** KUKA KR 500 R2830 (6-DOF, spherical wrist architecture)  
**Source of DH parameters:** RobotKinematicsCatalogue (`KUKA_KR500_R2830.py`)  
**Units:** angles in rad, lengths in mm

This document implements **Task A — Forward Kinematics (FK)** per the Project 1 instructions.

---

## 1. Task A Requirements Checklist
Per the project handout, Task A requires: :contentReference[oaicite:1]{index=1}
- [x] Assign coordinate frames following DH convention  
- [x] Derive individual transforms \,^{i-1}T_i for i = 1..6  
- [x] Compute overall tool transform \(T^{\text{tool}}_{\text{base}} = {}^{0}T_1 {}^{1}T_2 \cdots {}^{5}T_6\)  
- [ ] Validate against RoboDK for several joint configurations (see Section 6)

> Note: I am using the **Modified DH (DHM)** convention used by RobotKinematicsCatalogue, so \,^{i-1}T_i matches that implementation exactly.

---

## 2. Given Kinematic Parameters (Modified DHM)
The DHM table is stored per-joint as \([ \alpha_i, a_i, d_i, \theta_{0,i} ]\).

| i | \(\alpha_i\) [rad] | \(a_i\) [mm] | \(d_i\) [mm] | \(\theta_{0,i}\) [rad] |
|---:|---:|---:|---:|---:|
| 1 | \(0\) | 0 | 1045 | 0 |
| 2 | \(-\pi/2\) | 500 | 0 | 0 |
| 3 | \(0\) | 1300 | 0 | \(-\pi/2\) |
| 4 | \(-\pi/2\) | -55 | 1025 | 0 |
| 5 | \(+\pi/2\) | 0 | 0 | 0 |
| 6 | \(-\pi/2\) | 0 | 290 | \(+\pi\) |

### 2.1 Joint sign convention (as used by the catalogue)
The catalogue defines an “inverted joint” vector:
\[
s = [s_1,\ldots,s_6] = [-1,\ 1,\ 1,\ -1,\ 1,\ -1].
\]
Therefore the effective joint angle used in FK is:
\[
\theta_i(q_i)=\theta_{0,i} + s_i\,q_i.
\]
So explicitly:
\[
\theta_1=-q_1,\quad
\theta_2=q_2,\quad
\theta_3=q_3-\pi/2,\quad
\theta_4=-q_4,\quad
\theta_5=q_5,\quad
\theta_6=\pi-q_6.
\]

---

## 3. Modified-DH Homogeneous Transform (per joint)
For each joint \(i\), define:
- \(c_i=\cos(\theta_i)\), \(s_i=\sin(\theta_i)\)
- \(c_{\alpha i}=\cos(\alpha_i)\), \(s_{\alpha i}=\sin(\alpha_i)\)

Then the **Modified DH** transform from frame \(i-1\) to frame \(i\) is:

\[
{}^{i-1}\!T_i =
\begin{bmatrix}
c_i & -s_i & 0 & a_i\\
s_i c_{\alpha i} & c_i c_{\alpha i} & -s_{\alpha i} & -s_{\alpha i} d_i\\
s_i s_{\alpha i} & c_i s_{\alpha i} & c_{\alpha i} & c_{\alpha i} d_i\\
0&0&0&1
\end{bmatrix}.
\]

---

## 4. Individual Joint Transforms \(\,^{i-1}T_i\)
Below are the six transforms with the KR500 parameters substituted.

Let \(\theta_i\) be as defined in Section 2.1.

### \(^{0}T_1\)  (\(\alpha_1=0, a_1=0, d_1=1045\))
\[
{}^{0}\!T_1 =
\begin{bmatrix}
\cos\theta_1 & -\sin\theta_1 & 0 & 0\\
\sin\theta_1 & \cos\theta_1  & 0 & 0\\
0 & 0 & 1 & 1045\\
0&0&0&1
\end{bmatrix}.
\]

### \(^{1}T_2\)  (\(\alpha_2=-\pi/2, a_2=500, d_2=0\))
Using \(\cos(-\pi/2)=0\), \(\sin(-\pi/2)=-1\):
\[
{}^{1}\!T_2 =
\begin{bmatrix}
\cos\theta_2 & -\sin\theta_2 & 0 & 500\\
0 & 0 & 1 & 0\\
-\sin\theta_2 & -\cos\theta_2 & 0 & 0\\
0&0&0&1
\end{bmatrix}.
\]

### \(^{2}T_3\)  (\(\alpha_3=0, a_3=1300, d_3=0\))
\[
{}^{2}\!T_3 =
\begin{bmatrix}
\cos\theta_3 & -\sin\theta_3 & 0 & 1300\\
\sin\theta_3 & \cos\theta_3  & 0 & 0\\
0 & 0 & 1 & 0\\
0&0&0&1
\end{bmatrix}.
\]

### \(^{3}T_4\)  (\(\alpha_4=-\pi/2, a_4=-55, d_4=1025\))
\[
{}^{3}\!T_4 =
\begin{bmatrix}
\cos\theta_4 & -\sin\theta_4 & 0 & -55\\
0 & 0 & 1 & 1025\\
-\sin\theta_4 & -\cos\theta_4 & 0 & 0\\
0&0&0&1
\end{bmatrix}.
\]

### \(^{4}T_5\)  (\(\alpha_5=+\pi/2, a_5=0, d_5=0\))
Using \(\cos(\pi/2)=0\), \(\sin(\pi/2)=1\):
\[
{}^{4}\!T_5 =
\begin{bmatrix}
\cos\theta_5 & -\sin\theta_5 & 0 & 0\\
0 & 0 & -1 & 0\\
\sin\theta_5 & \cos\theta_5 & 0 & 0\\
0&0&0&1
\end{bmatrix}.
\]

### \(^{5}T_6\)  (\(\alpha_6=-\pi/2, a_6=0, d_6=290\))
\[
{}^{5}\!T_6 =
\begin{bmatrix}
\cos\theta_6 & -\sin\theta_6 & 0 & 0\\
0 & 0 & 1 & 290\\
-\sin\theta_6 & -\cos\theta_6 & 0 & 0\\
0&0&0&1
\end{bmatrix}.
\]

---

## 5. Overall Forward Kinematics
The full end-effector transform is the ordered product:

\[
{}^{0}\!T_6(q) = {}^{0}\!T_1\;{}^{1}\!T_2\;{}^{2}\!T_3\;{}^{3}\!T_4\;{}^{4}\!T_5\;{}^{5}\!T_6.
\]

Implementation note: in code, compute this by **numerical matrix multiplication** (do not hand-expand unless needed for a symbolic appendix). This is the standard, robust approach and matches the project expectation for Task A. :contentReference[oaicite:2]{index=2}

---

## 6. Sanity Checks (quick verification before RoboDK comparison)
### 6.1 Rotation validity
For any \(q\), the rotation should satisfy:
\[
R_{06}^T R_{06} \approx I,\qquad \det(R_{06}) \approx 1.
\]
Numerically, this should be within ~\(10^{-12}\) to \(10^{-15}\) depending on floating-point precision.

### 6.2 Zero configuration check
Set \(q=[0,0,0,0,0,0]\) rad.
Expected (from DH geometry + tool offset):
- Position approximately \(p_{06} \approx [3115,\ 0,\ 990]^T\) mm
- Rotation is a valid orthonormal matrix (det ~ 1)

If your computed result is wildly different (e.g., wrong sign on z, or magnitude not ~3.1 m), the most common causes are:
- forgetting the joint sign flips \(s_i\)
- forgetting \(\theta_{0,3}=-\pi/2\) and \(\theta_{0,6}=+\pi\)
- mixing standard DH with modified DH

---

## 7. Task A Validation Against RoboDK (required by handout)
The project instructions require RoboDK validation of FK (Task A). :contentReference[oaicite:3]{index=3}

### 7.1 Test plan
1. Choose 5–10 test joint configurations \(q^{(k)}\) (include:
   - all-zeros,
   - a few moderate random configs,
   - near-limit configs).
2. For each \(q^{(k)}\), compute:
   \[
   T^{(k)}_{\text{FK}} = {}^{0}\!T_6(q^{(k)}).
   \]
3. In RoboDK:
   - load the same KR500 model,
   - set the robot joints to \(q^{(k)}\),
   - read the tool pose \(T^{(k)}_{\text{RoboDK}}\).
4. Compare:
   - position error \( \|p_{\text{FK}}-p_{\text{RoboDK}}\| \) (mm)
   - orientation error (e.g., \( \|R_{\text{FK}}-R_{\text{RoboDK}}\|_F \) or angle error from \(R_{\text{err}}=R_{\text{FK}}^T R_{\text{RoboDK}}\)).

### 7.2 Reporting
Include either:
- screenshots of RoboDK with the joint values and tool pose visible, **and/or**
- a small validation table with numeric error metrics.

> If RoboDK pose readout is limited by licensing, use it at minimum as a **visual** confirmation and include a fallback numerical cross-check against an independent FK source (e.g., the catalogue FK script). However, RoboDK comparison is the stated ground-truth reference in the instructions. :contentReference[oaicite:4]{index=4}

---

## 8. Deliverable Produced by This Document
- DHM parameter table (KR500 R2830)
- Modified-DH transform definition
- Explicit per-joint transforms \(^{i-1}T_i\)
- Overall FK product \(^{0}T_6\)
- A clear validation procedure aligned to Task A :contentReference[oaicite:5]{index=5}
