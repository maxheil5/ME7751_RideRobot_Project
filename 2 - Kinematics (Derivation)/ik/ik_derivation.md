# Task B — Analytical Inverse Kinematics (IK)
**Robot:** KUKA KR 500 R2830 (6-DOF)  
**Kinematic type:** Industrial 6R with **spherical wrist** (decoupled position + orientation)  
**DH convention used:** **Modified DH (DHM)** matching the RobotKinematicsCatalogue table  
**Units:** angles in rad (internally), joint limits often specified in deg; lengths in mm

---

## 1) Task B Requirements (what this file covers)
This document provides:
1. Identification of the kinematic structure enabling a closed-form IK (spherical wrist decoupling).
2. Derivation of **all IK branches** (up to 8 solutions: shoulder L/R × elbow up/down × wrist flip/no-flip).
3. A closed-form analytical solution for $(q_1,\dots,q_6)$ from a given target pose ${}^{0}T_6$.
4. Singularity handling (especially wrist singularity at $q_5 \approx 0$ or $\pi$).
5. Joint-limit checking guidance.

---

## 2) Problem Statement and Pose Definition
Given a desired end-effector pose (tool/flange) relative to the base:

$$
{}^{0}T_6 =
\begin{bmatrix}
R_{06} & p_{06}\\
0\ 0\ 0 & 1
\end{bmatrix},
$$

where $R_{06}\in SO(3)$ and $p_{06}=[p_x\ p_y\ p_z]^T\in\mathbb{R}^3$ (mm),
we seek all joint configurations

$$
q = [q_1,\ q_2,\ q_3,\ q_4,\ q_5,\ q_6]^T
$$

that satisfy the FK equation:

$$
FK(q) = {}^{0}T_6.
$$

> Assumption: ${}^{0}T_6$ corresponds to the same “frame 6” as used in Task A FK (flange/tool frame with no additional tool transform). If you attach a tool frame $T_{6T}$, replace the target by ${}^{0}T_6 \leftarrow {}^{0}T_T\,T_{6T}^{-1}$ before running IK.

---

## 3) Robot Parameters and Conventions

### 3.1 Modified DHM parameters (KR500 R2830)
Each joint $i$ uses DHM parameters:

$$
[\alpha_i,\ a_i,\ d_i,\ \theta_{0,i}].
$$

| i | $\alpha_i$ [rad] | $a_i$ [mm] | $d_i$ [mm] | $\theta_{0,i}$ [rad] |
|---:|---:|---:|---:|---:|
| 1 | $0$ | $0$ | $1045$ | $0$ |
| 2 | $-\pi/2$ | $500$ | $0$ | $0$ |
| 3 | $0$ | $1300$ | $0$ | $-\pi/2$ |
| 4 | $-\pi/2$ | $-55$ | $1025$ | $0$ |
| 5 | $+\pi/2$ | $0$ | $0$ | $0$ |
| 6 | $-\pi/2$ | $0$ | $290$ | $+\pi$ |

### 3.2 Joint sign convention + offsets (what enters the per-link transform)
The effective joint angle used in each per-link transform is:

$$
\theta_i = \theta_{0,i} + s_i\,q_i,
$$

with

$$
s = [s_1,\dots,s_6] = [-1,\ 1,\ 1,\ -1,\ 1,\ -1].
$$

So explicitly:

$$
\theta_1=-q_1,\quad
\theta_2=q_2,\quad
\theta_3=q_3-\pi/2,\quad
\theta_4=-q_4,\quad
\theta_5=q_5,\quad
\theta_6=\pi-q_6.
$$

> IK will solve for the angles $q_i$ (RoboDK joint angles). Internally, the Modified-DH transforms use $\theta_i$ above.

---

## 4) Structure Enabling Analytical IK: Spherical Wrist Decoupling
A spherical wrist means the last three joint axes intersect at a single point, so we can decouple:

1) **Position IK**: solve $(q_1,q_2,q_3)$ from the **wrist center** position.  
2) **Orientation IK**: solve $(q_4,q_5,q_6)$ from the remaining wrist rotation.

For this robot, joints 4–6 are a wrist with no translation between frames 4 and 5 (since $a_5=d_5=0$), and the final tool offset is $d_6=290$ mm.

---

## 5) Step 1 — Compute the Wrist Center $p_{wc}$
Let the tool z-axis (in base coordinates) be the third column of $R_{06}$:

$$
\hat{z}_6 = R_{06}
\begin{bmatrix}
0\\0\\1
\end{bmatrix}.
$$

Because the last link has $d_6=290$ mm along $\hat{z}_6$, the wrist center is:

$$
p_{wc} = p_{06} - d_6\,\hat{z}_6
\quad\text{with}\quad d_6=290.
$$

Write $p_{wc}=[x_w,\ y_w,\ z_w]^T$.

---

## 6) Step 2 — Solve Shoulder (Base) Angle $q_1$ (2 branches)

From Task A FK, the wrist center satisfies:

$$
x_w = (\cos q_1)\,B,\qquad y_w = -(\sin q_1)\,B,
$$

for some scalar $B$ (the in-plane “arm radius”, which may be positive or negative depending on branch).

Define:

$$
\rho = \sqrt{x_w^2+y_w^2}.
$$

Two shoulder branches:

### Branch S1 (shoulder “primary”)
$$
q_{1}^{(a)} = \operatorname{atan2}(-y_w,\ x_w),
\qquad B^{(a)} = +\rho.
$$

### Branch S2 (shoulder “flipped”)
$$
q_{1}^{(b)} = q_{1}^{(a)}+\pi,
\qquad B^{(b)} = -\rho.
$$

> Using $B=\pm\rho$ is what preserves both shoulder configurations.

---

## 7) Step 3 — Solve Elbow/Arm Angles $(q_2,q_3)$ (2 branches)

### 7.1 Reduce to a planar 2R problem
From Task A FK (wrist center position), the following relations hold:

$$
B = 500 + 1300\cos q_2 + 1025\cos(q_2+q_3) - 55\sin(q_2+q_3),
$$

$$
z_w = 1045 - 1300\sin q_2 - 1025\sin(q_2+q_3) - 55\cos(q_2+q_3).
$$

For each shoulder branch (each chosen $B$), define shifted planar coordinates:

$$
r = B - 500,
\qquad
z = 1045 - z_w.
$$

Then we can write:

$$
r = 1300\cos q_2 + 1025\cos(q_2+q_3) - 55\sin(q_2+q_3),
$$

$$
z = 1300\sin q_2 + 1025\sin(q_2+q_3) + 55\cos(q_2+q_3).
$$

Now combine the $(1025,\ 55)$ terms into a single link length:

$$
L = \sqrt{1025^2 + 55^2},
\qquad
\phi = \operatorname{atan2}(55,\ 1025).
$$

Using trig identities:

$$
1025\cos(q_2+q_3) - 55\sin(q_2+q_3) = L\cos\big((q_2+q_3)+\phi\big),
$$

$$
1025\sin(q_2+q_3) + 55\cos(q_2+q_3) = L\sin\big((q_2+q_3)+\phi\big).
$$

So the planar system becomes:

$$
r = 1300\cos q_2 + L\cos\big(q_2+q_3+\phi\big),
$$

$$
z = 1300\sin q_2 + L\sin\big(q_2+q_3+\phi\big).
$$

This is a standard 2R arm with:
- link 1 length $A = 1300$,
- link 2 length $L$,
- elbow internal angle $\gamma = q_3+\phi$.

### 7.2 Closed-form elbow solution (2 branches)
Define:

$$
D = \frac{r^2+z^2 - A^2 - L^2}{2AL},
\qquad A=1300.
$$

Feasibility condition (reachability):

$$
|D|\le 1.
$$

Elbow angle branches:

$$
\gamma^{(\pm)} = \operatorname{atan2}\big(\pm\sqrt{1-D^2},\ D\big).
$$

Then:

$$
q_2^{(\pm)} = \operatorname{atan2}(z,\ r)\;-\;\operatorname{atan2}\big(L\sin\gamma^{(\pm)},\ A + L\cos\gamma^{(\pm)}\big),
$$

$$
q_3^{(\pm)} = \gamma^{(\pm)} - \phi.
$$

> For each shoulder branch (2) and elbow branch (2), you get up to **4** candidate solutions for $(q_1,q_2,q_3)$.

---

## 8) Step 4 — Compute Wrist Rotation $R_{36}$
Once a candidate $(q_1,q_2,q_3)$ is known, compute the rotation from base to frame 3:

Let:

$$
c1=\cos q_1,\ s1=\sin q_1,\quad
c_{23}=\cos(q_2+q_3),\ s_{23}=\sin(q_2+q_3).
$$

Then the closed-form $R_{03}$ is:

$$
R_{03} =
\begin{bmatrix}
s_{23}c1 & c_{23}c1 & s1\\
-s_{23}s1 & -c_{23}s1 & c1\\
c_{23} & -s_{23} & 0
\end{bmatrix}.
$$

Now compute:

$$
R_{36} = R_{03}^T\,R_{06}.
$$

This isolates the wrist rotation that must be achieved by joints 4–6.

---

## 9) Step 5 — Solve Wrist Angles $(q_4,q_5,q_6)$ (2 branches)

### 9.1 Wrist parameterization in terms of $\theta_4,\theta_5,\theta_6$
Recall:

$$
\theta_4=-q_4,\qquad \theta_5=q_5,\qquad \theta_6=\pi-q_6.
$$

For this robot’s Modified-DH wrist, the rotation $R_{36}$ has the following structure (derived from $R_{36}=R_{34}R_{45}R_{56}$):

$$
R_{36} =
\begin{bmatrix}
\cdot & \cdot & -\cos\theta_4\,\sin\theta_5\\
\cos\theta_6\,\sin\theta_5 & -\sin\theta_5\,\sin\theta_6 & \cos\theta_5\\
\cdot & \cdot & \sin\theta_4\,\sin\theta_5
\end{bmatrix}.
$$

From this, we can extract $\theta_5$ immediately:

$$
\cos\theta_5 = R_{36}(2,3).
$$

Define:

$$
s5 = \sqrt{R_{36}(2,1)^2 + R_{36}(2,2)^2}.
$$

Then one wrist branch uses $\sin\theta_5 = +s5$ and the other uses $\sin\theta_5=-s5$.

### 9.2 Non-singular case ($s5 > \varepsilon$)
If $s5$ is not near zero:

**Branch W1 (no wrist flip):**
$$
\theta_5^{(a)} = \operatorname{atan2}(s5,\ R_{36}(2,3)),
$$
$$
\theta_4^{(a)} = \operatorname{atan2}\big(R_{36}(3,3),\ -R_{36}(1,3)\big),
$$
$$
\theta_6^{(a)} = \operatorname{atan2}\big(-R_{36}(2,2),\ R_{36}(2,1)\big).
$$

**Branch W2 (wrist flip):**
$$
\theta_5^{(b)} = \operatorname{atan2}(-s5,\ R_{36}(2,3)) = -\theta_5^{(a)},
$$
$$
\theta_4^{(b)} = \theta_4^{(a)} + \pi,
\qquad
\theta_6^{(b)} = \theta_6^{(a)} + \pi.
$$

Finally map back to RoboDK joint angles:

$$
q_4 = -\theta_4,\qquad q_5=\theta_5,\qquad q_6=\pi-\theta_6.
$$

### 9.3 Wrist singularity ($s5 \le \varepsilon$)
When $s5\approx 0$, the wrist loses a DOF and $\theta_4$ and $\theta_6$ become coupled.

- If $\cos\theta_5 = R_{36}(2,3)\approx +1$, then $\theta_5 \approx 0$ and only the sum
  $$\psi = \theta_4 + \theta_6$$
  is observable. From the reduced form:
  $$\psi = \operatorname{atan2}\big(-R_{36}(1,2),\ R_{36}(1,1)\big).$$
  A simple choice is $\theta_4=0$, $\theta_6=\psi$.

- If $\cos\theta_5 = R_{36}(2,3)\approx -1$, then $\theta_5 \approx \pi$ and only the difference
  $$\psi = \theta_4 - \theta_6$$
  is observable. One consistent extraction is:
  $$\psi = \operatorname{atan2}\big(-R_{36}(1,2),\ -R_{36}(1,1)\big).$$
  A simple choice is $\theta_4=0$, $\theta_6=-\psi$.

Then map to $q_4,q_5,q_6$ using $q_4=-\theta_4$, $q_5=\theta_5$, $q_6=\pi-\theta_6$.

> Any other split of the coupled angle (e.g., $\theta_4=\psi/2$, $\theta_6=\psi/2$) is also valid; choose what best respects joint limits.

---

## 10) Full Branch Enumeration (up to 8 solutions)

We enumerate solutions as:

- **2 shoulder branches:** $(q_1^{(a)},B^{(a)})$ and $(q_1^{(b)},B^{(b)})$
- **2 elbow branches:** $(q_2^{(+)},q_3^{(+)})$ and $(q_2^{(-)},q_3^{(-)})$
- **2 wrist branches:** W1 (no-flip) and W2 (flip)

Total candidates:

$$
N_{\max} = 2\times 2\times 2 = 8.
$$

For each candidate:
1. Compute $p_{wc}$ from the target pose.
2. Choose shoulder $(q_1,B)$.
3. Solve elbow $(q_2,q_3)$ using the corresponding $(r,z)$.
4. Compute $R_{03}$ and $R_{36}=R_{03}^T R_{06}$.
5. Solve wrist $(q_4,q_5,q_6)$ (two branches unless singular).
6. Normalize angles and filter by joint limits.

---

## 11) Joint Limits and Normalization
After computing each candidate $q$:
1. Wrap angles into a consistent range (e.g., $(-\pi,\pi]$).
2. If needed, add/subtract $2\pi$ to bring angles into the robot’s allowable joint ranges.

Typical joint limits for this model (deg) are:
- $q_1 \in [-185,185]$
- $q_2 \in [-130,20]$
- $q_3 \in [-100,144]$
- $q_4 \in [-350,350]$
- $q_5 \in [-120,120]$
- $q_6 \in [-350,350]$

> If multiple wraps satisfy limits, keep the one closest to a reference configuration (e.g., RoboDK home) to avoid discontinuities.

---

## 12) Final Closed-form Analytical Solution (summary)

Given ${}^{0}T_6=[R_{06},p_{06}]$:

**(A) Wrist center**
$$
p_{wc} = p_{06} - 290\,R_{06}\begin{bmatrix}0\\0\\1\end{bmatrix}
= \begin{bmatrix}x_w\\y_w\\z_w\end{bmatrix}.
$$

**(B) Shoulder (2 branches)**
$$
\rho = \sqrt{x_w^2+y_w^2},
\quad
q_1^{(a)}=\operatorname{atan2}(-y_w,x_w),\ B^{(a)}=+\rho,
\quad
q_1^{(b)}=q_1^{(a)}+\pi,\ B^{(b)}=-\rho.
$$

**(C) Elbow (2 branches per shoulder)**
$$
r = B - 500,\quad z = 1045 - z_w,
\quad
L=\sqrt{1025^2+55^2},\quad \phi=\operatorname{atan2}(55,1025),\quad A=1300,
$$
$$
D=\frac{r^2+z^2-A^2-L^2}{2AL},\quad |D|\le 1,
$$
$$
\gamma^{(\pm)}=\operatorname{atan2}\big(\pm\sqrt{1-D^2},D\big),
$$
$$
q_2^{(\pm)}=\operatorname{atan2}(z,r)-\operatorname{atan2}\big(L\sin\gamma^{(\pm)},A+L\cos\gamma^{(\pm)}\big),
\quad
q_3^{(\pm)}=\gamma^{(\pm)}-\phi.
$$

**(D) Wrist rotation**
$$
R_{03}=
\begin{bmatrix}
s_{23}c1 & c_{23}c1 & s1\\
-s_{23}s1 & -c_{23}s1 & c1\\
c_{23} & -s_{23} & 0
\end{bmatrix},
\quad
R_{36}=R_{03}^T R_{06},
$$
with $c1=\cos q_1$, $s1=\sin q_1$, $c_{23}=\cos(q_2+q_3)$, $s_{23}=\sin(q_2+q_3)$.

**(E) Wrist angles (2 branches unless singular)**
$$
c5 = R_{36}(2,3),
\quad
s5=\sqrt{R_{36}(2,1)^2+R_{36}(2,2)^2}.
$$

If $s5>\varepsilon$:
$$
\theta_5^{(a)}=\operatorname{atan2}(s5,c5),\quad
\theta_4^{(a)}=\operatorname{atan2}\big(R_{36}(3,3),-R_{36}(1,3)\big),\quad
\theta_6^{(a)}=\operatorname{atan2}\big(-R_{36}(2,2),R_{36}(2,1)\big),
$$
wrist flip:
$$
\theta_5^{(b)}=-\theta_5^{(a)},\quad \theta_4^{(b)}=\theta_4^{(a)}+\pi,\quad \theta_6^{(b)}=\theta_6^{(a)}+\pi.
$$

Map to RoboDK joints:
$$
q_4=-\theta_4,\qquad q_5=\theta_5,\qquad q_6=\pi-\theta_6.
$$

If $s5\le \varepsilon$, use the singular-case rules in Section 9.3.

---

## 13) What you should implement in code (Task B code hook)
Your IK function should:
- Input: $R_{06}$, $p_{06}$ (or full ${}^{0}T_6$)
- Output: a list of solutions $\{q^{(k)}\}$, $k=1,\dots,N$ with $N\le 8$
- Filter: unreachable ($|D|>1$), wrist singular handling, joint limits
- Prefer: optionally sort solutions by distance to a seed configuration (e.g., RoboDK home)
