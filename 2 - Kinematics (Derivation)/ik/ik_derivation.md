# Task B — Analytical Inverse Kinematics (IK)
**Robot:** KUKA KR 500 R2830 (6-DOF)  
**Kinematic type:** Industrial 6R with a **spherical wrist** (position/orientation decoupling)  
**DH convention used:** **Modified DH (DHM)** (same convention as Task A FK)  
**Units:** angles in rad (internally), lengths in mm

---

## 1) What this document provides (Task B)
This document provides:
- A full analytical IK derivation and final closed-form solution for $q_1,\dots,q_6$
- Enumeration of **all branches** (up to 8: shoulder × elbow × wrist)
- Explicit handling of wrist singularities ($\sin\theta_5 \approx 0$)
- Guidance for joint-limit filtering and angle wrapping

---

## 2) Problem statement
Given a desired end-effector pose (frame 6) relative to the base:

$$
{}^{0}T_6 =
\begin{bmatrix}
R_{06} & p_{06}\\
0\ 0\ 0 & 1
\end{bmatrix},
$$

where $R_{06}\in SO(3)$ and $p_{06}=[p_x\ p_y\ p_z]^T$ (mm), find all joint vectors

$$
q = [q_1,\ q_2,\ q_3,\ q_4,\ q_5,\ q_6]^T
$$

such that $FK(q)={}^0T_6$.

> Assumption: the target pose corresponds to the same frame-6 definition used in Task A FK (no extra tool transform). If you attach a tool frame $T_{6T}$, replace the target by ${}^{0}T_6 \leftarrow {}^{0}T_T\,T_{6T}^{-1}$ before running IK.

---

## 3) Robot parameters and conventions

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
The per-link transform uses the effective angles

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

---

## 4) Why closed-form IK is possible (spherical wrist decoupling)
This robot has a spherical wrist (last three joint axes intersect). Therefore:

1. **Position IK:** solve $(q_1,q_2,q_3)$ using the wrist center position $p_{wc}$.  
2. **Orientation IK:** solve $(q_4,q_5,q_6)$ using a reduced wrist rotation $R_{36}$.

---

## 5) Step 1 — Wrist center $p_{wc}$
Let the tool $z$-axis (in base coordinates) be the third column of $R_{06}$:

$$
\hat{z}_6 = R_{06}
\begin{bmatrix}
0\\0\\1
\end{bmatrix}.
$$

Because the final offset is $d_6=290$ mm along $\hat{z}_6$, the wrist center is:

$$
p_{wc} = p_{06} - d_6\,\hat{z}_6,
\qquad d_6 = 290.
$$

Write:

$$
p_{wc}=
\begin{bmatrix}
x_w\\y_w\\z_w
\end{bmatrix}.
$$

---

## 6) Step 2 — Solve shoulder (base) angle $q_1$ (2 branches)

From the FK wrist-center structure, we can write:

$$
x_w = (\cos q_1)\,B,\qquad y_w = -(\sin q_1)\,B,
$$

for some scalar $B$ (the in-plane “arm radius”) which can be positive or negative depending on the shoulder branch.

Define:

$$
\rho = \sqrt{x_w^2+y_w^2}.
$$

Two shoulder branches:

### Branch S1 (shoulder “primary”)

$$
q_1^{(a)} = \mathrm{atan2}(-y_w,\ x_w),
\qquad
B^{(a)} = +\rho.
$$

### Branch S2 (shoulder “flipped”)

$$
q_1^{(b)} = q_1^{(a)}+\pi,
\qquad
B^{(b)} = -\rho.
$$

> Using $B=\pm\rho$ is what preserves both shoulder configurations.

---

## 7) Step 3 — Solve elbow/arm angles $(q_2,q_3)$ (2 branches)

### 7.1 Reduce to a planar 2R problem
From the FK wrist-center position:

$$
B = 500 + 1300\cos q_2 + 1025\cos(q_2 + q_3) - 55\sin(q_2 + q_3),
$$

$$
z_w = 1045 - 1300\sin q_2 - 1025\sin(q_2 + q_3) - 55\cos(q_2 + q_3).
$$

For each shoulder branch (each chosen $B$), define shifted planar coordinates:

$$
r = B - 500,
\qquad
z = 1045 - z_w.
$$

Then:

$$
r = 1300\cos q_2 + 1025\cos(q_2 + q_3) - 55\sin(q_2 + q_3),
$$

$$
z = 1300\sin q_2 + 1025\sin(q_2 + q_3) + 55\cos(q_2 + q_3).
$$

Combine the $(1025,55)$ terms into a single link length:

$$
L = \sqrt{1025^2 + 55^2},
\qquad
\phi = \mathrm{atan2}(55,\ 1025).
$$

Using trig identities:

$$
1025\cos(q_2+q_3) - 55\sin(q_2+q_3) = L\cos\!\left((q_2+q_3)+\phi\right),
$$

$$
1025\sin(q_2+q_3) + 55\cos(q_2+q_3) = L\sin\!\left((q_2+q_3)+\phi\right).
$$

So:

$$
r = 1300\cos q_2 + L\cos\!\left(q_2+q_3+\phi\right),
$$

$$
z = 1300\sin q_2 + L\sin\!\left(q_2+q_3+\phi\right).
$$

Define the planar 2R constants:
- $A = 1300$
- $\gamma = q_3+\phi$

### 7.2 Closed-form elbow solution (2 branches)
Define:

$$
D = \frac{r^2+z^2-A^2-L^2}{2AL},
\qquad A=1300.
$$

Reachability condition:

$$
|D|\le 1.
$$

Elbow branches:

$$
\gamma^{(\pm)} = \mathrm{atan2}\!\left(\pm\sqrt{1-D^2},\ D\right).
$$

Then:


$q_2^{(\pm)} = \mathrm{atan2}(z,\ r)$
$\mathrm{atan2}\!\left(L\sin\gamma^{(\pm)},\ A + L\cos\gamma^{(\pm)}\right)$,


$$
q_3^{(\pm)} = \gamma^{(\pm)} - \phi.
$$

> For each shoulder branch (2) and elbow branch (2), you get up to 4 candidate solutions for $(q_1,q_2,q_3)$.

---

## 8) Step 4 — Compute wrist rotation $R_{36}$
Once a candidate $(q_1,q_2,q_3)$ is known, compute:

$$
R_{36} = R_{03}^T\,R_{06}.
$$

A closed-form $R_{03}$ is:

Let

$$
c1=\cos q_1,\ s1=\sin q_1,\quad
c_{23}=\cos(q_2+q_3),\ s_{23}=\sin(q_2+q_3).
$$

Then

$$
R_{03} =
\begin{bmatrix}
s_{23}c1 & c_{23}c1 & s1\\
-s_{23}s1 & -c_{23}s1 & c1\\
c_{23} & -s_{23} & 0
\end{bmatrix}.
$$

---

## 9) Step 5 — Solve wrist angles $(q_4,q_5,q_6)$ (2 branches)

### 9.1 Solve $(\theta_4,\theta_5,\theta_6)$ then map to $(q_4,q_5,q_6)$
Recall:

$$
\theta_4=-q_4,\qquad \theta_5=q_5,\qquad \theta_6=\pi-q_6.
$$

We solve $(\theta_4,\theta_5,\theta_6)$ from $R_{36}$, then map to RoboDK joints.

### 9.2 Non-singular case ($s5 > \varepsilon$)
Compute:

$$
c5 = R_{36}(2,3),
\qquad
s5 = \sqrt{R_{36}(2,1)^2 + R_{36}(2,2)^2}.
$$

If $s5 > \varepsilon$ (e.g., $\varepsilon=10^{-8}$), then:

**Branch W1 (no wrist flip):**

$$
\theta_5^{(a)} = \mathrm{atan2}(s5,\ c5),
$$

$$
\theta_4^{(a)} = \mathrm{atan2}\!\left(R_{36}(3,3),\ -R_{36}(1,3)\right),
$$

$$
\theta_6^{(a)} = \mathrm{atan2}\!\left(-R_{36}(2,2),\ R_{36}(2,1)\right).
$$

**Branch W2 (wrist flip):**

$$
\theta_5^{(b)} = -\theta_5^{(a)},
\qquad
\theta_4^{(b)} = \theta_4^{(a)} + \pi,
\qquad
\theta_6^{(b)} = \theta_6^{(a)} + \pi.
$$

Map to RoboDK joints:

$$
q_4=-\theta_4,\qquad q_5=\theta_5,\qquad q_6=\pi-\theta_6.
$$

### 9.3 Wrist singularity ($s5 \le \varepsilon$)
When $s5 \approx 0$, the wrist loses a DOF and $\theta_4$ and $\theta_6$ become coupled.

- If $c5=R_{36}(2,3)\approx +1$, then $\theta_5\approx 0$ and only the sum
  $$
  \psi=\theta_4+\theta_6
  $$
  is observable. One consistent extraction is:
  $$
  \psi = \mathrm{atan2}\!\left(-R_{36}(1,2),\ R_{36}(1,1)\right).
  $$
  A simple choice is $\theta_4=0$, $\theta_6=\psi$.

- If $c5=R_{36}(2,3)\approx -1$, then $\theta_5\approx \pi$ and only the difference
  $$
  \psi=\theta_4-\theta_6
  $$
  is observable. One consistent extraction is:
  $$
  \psi = \mathrm{atan2}\!\left(-R_{36}(1,2),\ -R_{36}(1,1)\right).
  $$
  A simple choice is $\theta_4=0$, $\theta_6=-\psi$.

Then map to RoboDK joints:

$$
q_4=-\theta_4,\qquad q_5=\theta_5,\qquad q_6=\pi-\theta_6.
$$

> Any other split of the coupled angle is also valid; choose what best respects joint limits.

---

## 10) Full branch enumeration (up to 8 solutions)
- Shoulder: 2 branches (S1, S2)
- Elbow: 2 branches per shoulder ($+$, $-$)
- Wrist: 2 branches per arm solution (W1, W2) unless singular

Total candidates:

$$
N_{\max}=2\times 2\times 2 = 8.
$$

For each candidate:
1. Compute $p_{wc}$.
2. Choose shoulder $(q_1,B)$.
3. Solve elbow $(q_2,q_3)$ via $(r,z)$ and $D$.
4. Compute $R_{36}=R_{03}^T R_{06}$.
5. Solve wrist $(q_4,q_5,q_6)$ (or singular-case logic).
6. Normalize and filter by joint limits.

---

## 11) Joint limits, normalization, and filtering
After computing each candidate solution:
- Wrap angles into a consistent range (e.g., $(-\pi,\pi]$)
- Add/subtract $2\pi$ when needed to satisfy the robot joint limits
- Reject candidates outside limits

Typical joint limits (deg) for this model:
- $q_1 \in [-185,185]$
- $q_2 \in [-130,20]$
- $q_3 \in [-100,144]$
- $q_4 \in [-350,350]$
- $q_5 \in [-120,120]$
- $q_6 \in [-350,350]$

If multiple wraps satisfy limits, keep the one closest to a reference configuration (e.g., RoboDK home) to avoid discontinuities.

---

## 12) Final closed-form analytical solution (summary)

### (A) Wrist center

$$
p_{wc} = p_{06} - 290\,R_{06}\begin{bmatrix}0\\0\\1\end{bmatrix}
= \begin{bmatrix}x_w\\y_w\\z_w\end{bmatrix}.
$$

### (B) Shoulder (2 branches)

$$
\rho=\sqrt{x_w^2+y_w^2},
$$

$$
q_1^{(a)}=\mathrm{atan2}(-y_w,x_w),\quad B^{(a)}=+\rho,
\qquad
q_1^{(b)}=q_1^{(a)}+\pi,\quad B^{(b)}=-\rho.
$$

### (C) Elbow (2 branches per shoulder)

$$
r=B-500,\quad z=1045-z_w,
$$

$$
L=\sqrt{1025^2+55^2},\quad \phi=\mathrm{atan2}(55,1025),\quad A=1300,
$$

$$
D=\frac{r^2+z^2-A^2-L^2}{2AL},\quad |D|\le 1,
$$

$$
\gamma^{(\pm)}=\mathrm{atan2}\!\left(\pm\sqrt{1-D^2},D\right),
$$

$$
q_2^{(\pm)}=\mathrm{atan2}(z,r)-\mathrm{atan2}\!\left(L\sin\gamma^{(\pm)},A+L\cos\gamma^{(\pm)}\right),
\quad
q_3^{(\pm)}=\gamma^{(\pm)}-\phi.
$$

### (D) Wrist rotation

$$
R_{36}=R_{03}^T R_{06},
$$

with

$$
R_{03} =
\begin{bmatrix}
s_{23}c1 & c_{23}c1 & s1\\
-s_{23}s1 & -c_{23}s1 & c1\\
c_{23} & -s_{23} & 0
\end{bmatrix},
$$

and $c1=\cos q_1$, $s1=\sin q_1$, $c_{23}=\cos(q_2+q_3)$, $s_{23}=\sin(q_2+q_3)$.

### (E) Wrist angles (2 branches unless singular)

$$
c5 = R_{36}(2,3),\qquad s5=\sqrt{R_{36}(2,1)^2+R_{36}(2,2)^2}.
$$

If $s5>\varepsilon$:

$$
\theta_5^{(a)}=\mathrm{atan2}(s5,c5),
\quad
\theta_4^{(a)}=\mathrm{atan2}\!\left(R_{36}(3,3),-R_{36}(1,3)\right),
\quad
\theta_6^{(a)}=\mathrm{atan2}\!\left(-R_{36}(2,2),R_{36}(2,1)\right),
$$

wrist flip:

$$
\theta_5^{(b)}=-\theta_5^{(a)},\quad
\theta_4^{(b)}=\theta_4^{(a)}+\pi,\quad
\theta_6^{(b)}=\theta_6^{(a)}+\pi.
$$

Map to RoboDK joints:

$$
q_4=-\theta_4,\qquad q_5=\theta_5,\qquad q_6=\pi-\theta_6.
$$

If $s5\le\varepsilon$, use the singular-case rules in Section 9.3.

---

## 13) Implementation note (what your IK function should return)
Your IK function should:
- Input: ${}^{0}T_6$ (or $R_{06},p_{06}$)
- Output: all valid solutions $\{q^{(k)}\}$, $k=1,\dots,N$, with $N\le 8$
- Reject unreachable targets ($|D|>1$)
- Handle wrist singularities ($s5\approx 0$)
- Filter by joint limits
- Optionally sort solutions by distance to a seed configuration
