# Task A — Forward Kinematics (FK)
**Robot:** KUKA KR 500 R2830 (6-DOF)  
**DH convention:** **Modified DH (DHM)** (matches RobotKinematicsCatalogue format)  
**Units:** angles in **rad**, lengths in **mm**

This document is the Task A (FK) writeup: define the DHM parameters, define the per-link modified-DH transform, write out the six individual transforms, and define the overall FK chain for implementation + validation.

---

## 1) Task A Deliverables (what this file covers)
- Coordinate frame assumptions + notation (high-level)
- DHM parameter table and joint sign/offset convention
- Modified-DH transform definition (the exact matrix form used in code)
- Individual joint transforms ${}^{i-1}T_i$ for $i=1,\dots,6$
- Overall FK product ${}^{0}T_6(q)$
- Sanity checks + validation plan (RoboDK or alternate reference)

---

## 2) Notation and Frames
- Base frame: $\{0\}$
- Link frames: $\{1\},\{2\},\dots,\{6\}$
- FK output: ${}^{0}T_6(q)$ (pose of frame 6 expressed in base frame 0)

A homogeneous transform is written as:
${}^{0}T_6 = \begin{bmatrix} R_{06} & p_{06} \\ 0\ 0\ 0 & 1 \end{bmatrix},$
where $R_{06}\in SO(3)$ and $p_{06}\in\mathbb{R}^3$ (mm).

---

## 3) Given Kinematic Parameters (Modified DHM)
Each joint $i$ uses DHM parameters:
$$
[\alpha_i,\ a_i,\ d_i,\ \theta_{0,i}].
$$

### 3.1 DHM table (KR500 R2830)
| i | $\alpha_i$ [rad] | $a_i$ [mm] | $d_i$ [mm] | $\theta_{0,i}$ [rad] |
|---:|---:|---:|---:|---:|
| 1 | $0$ | $0$ | $1045$ | $0$ |
| 2 | $-\pi/2$ | $500$ | $0$ | $0$ |
| 3 | $0$ | $1300$ | $0$ | $-\pi/2$ |
| 4 | $-\pi/2$ | $-55$ | $1025$ | $0$ |
| 5 | $+\pi/2$ | $0$ | $0$ | $0$ |
| 6 | $-\pi/2$ | $0$ | $290$ | $+\pi$ |

### 3.2 Joint sign convention + offsets (what enters FK)
The effective angle used in the per-link transform is:
$$
\theta_i(q_i) = \theta_{0,i} + s_i\,q_i,
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

## 4) Modified DH Transform Definition
Define shorthand:
$$
c_i = \cos(\theta_i),\quad s_i=\sin(\theta_i),\quad
c_{\alpha i}=\cos(\alpha_i),\quad s_{\alpha i}=\sin(\alpha_i).
$$

Then the Modified DH transform from frame $(i-1)$ to frame $i$ is:
$$
{}^{i-1}T_i =
\begin{bmatrix}
c_i & -s_i & 0 & a_i \\
s_i c_{\alpha i} & c_i c_{\alpha i} & -s_{\alpha i} & -s_{\alpha i}\,d_i \\
s_i s_{\alpha i} & c_i s_{\alpha i} & c_{\alpha i} & c_{\alpha i}\,d_i \\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

> Important: This is **Modified DH**, not Standard DH. Do not swap conventions mid-derivation.

---

## 5) Individual Joint Transforms (${}^{i-1}T_i$)
Below are the six transforms with the KR500 parameters substituted.  
Let $\theta_i$ be defined in Section 3.2.

### 5.1 ${}^{0}T_1$  ($\alpha_1=0,\ a_1=0,\ d_1=1045$)
Since $\cos 0 = 1$, $\sin 0 = 0$:
$$
{}^{0}T_1 =
\begin{bmatrix}
\cos(\theta_1) & -\sin(\theta_1) & 0 & 0 \\
\sin(\theta_1) & \cos(\theta_1) & 0 & 0 \\
0 & 0 & 1 & 1045 \\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

### 5.2 ${}^{1}T_2$  ($\alpha_2=-\pi/2,\ a_2=500,\ d_2=0$)
Use $\cos(-\pi/2)=0$, $\sin(-\pi/2)=-1$:
$$
{}^{1}T_2 =
\begin{bmatrix}
\cos(\theta_2) & -\sin(\theta_2) & 0 & 500 \\
0 & 0 & 1 & 0 \\
-\sin(\theta_2) & -\cos(\theta_2) & 0 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

### 5.3 ${}^{2}T_3$  ($\alpha_3=0,\ a_3=1300,\ d_3=0$)
$$
{}^{2}T_3 =
\begin{bmatrix}
\cos(\theta_3) & -\sin(\theta_3) & 0 & 1300 \\
\sin(\theta_3) & \cos(\theta_3) & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

### 5.4 ${}^{3}T_4$  ($\alpha_4=-\pi/2,\ a_4=-55,\ d_4=1025$)
Use $\cos(-\pi/2)=0$, $\sin(-\pi/2)=-1$:
$$
{}^{3}T_4 =
\begin{bmatrix}
\cos(\theta_4) & -\sin(\theta_4) & 0 & -55 \\
0 & 0 & 1 & 1025 \\
-\sin(\theta_4) & -\cos(\theta_4) & 0 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

### 5.5 ${}^{4}T_5$  ($\alpha_5=+\pi/2,\ a_5=0,\ d_5=0$)
Use $\cos(\pi/2)=0$, $\sin(\pi/2)=1$:
$$
{}^{4}T_5 =
\begin{bmatrix}
\cos(\theta_5) & -\sin(\theta_5) & 0 & 0 \\
0 & 0 & -1 & 0 \\
\sin(\theta_5) & \cos(\theta_5) & 0 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

### 5.6 ${}^{5}T_6$  ($\alpha_6=-\pi/2,\ a_6=0,\ d_6=290$)
Use $\cos(-\pi/2)=0$, $\sin(-\pi/2)=-1$:
$$
{}^{5}T_6 =
\begin{bmatrix}
\cos(\theta_6) & -\sin(\theta_6) & 0 & 0 \\
0 & 0 & 1 & 290 \\
-\sin(\theta_6) & -\cos(\theta_6) & 0 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

---

## 6) Overall Forward Kinematics
The full FK is the ordered product:
$$
{}^{0}T_6(q) = {}^{0}T_1\;{}^{1}T_2\;{}^{2}T_3\;{}^{3}T_4\;{}^{4}T_5\;{}^{5}T_6.
$$

### 6.1 Practical implementation note
Do **not** hand-expand the final matrix unless you specifically need a symbolic closed form.  
In software, compute ${}^{0}T_6$ via numerical matrix multiplication:
1. Compute each $\theta_i(q_i)$ (include offsets and sign flips).
2. Build each ${}^{i-1}T_i$ from Section 5.
3. Multiply in order to obtain ${}^{0}T_6$.

---

## 7) Quick Sanity Checks (before full validation)
These checks catch the most common DH mistakes (wrong convention, missing offsets, sign errors).

### 7.1 Rotation validity
For any joint configuration:
- $R_{06}^T R_{06} \approx I$
- $\det(R_{06}) \approx 1$

### 7.2 Zero configuration check
Set $q=[0,0,0,0,0,0]$ (rad). With the given DHM geometry, the computed end-effector position should be on the order of a few meters in $x$ and roughly $\sim 1$ meter in $z$.

A quick numerical evaluation gives approximately:
$$
p_{06}(0) \approx \begin{bmatrix}3115\\0\\990\end{bmatrix}\ \text{mm}.
$$

If you get something wildly different (wrong magnitude or flipped axes), check:
- You used **Modified DH** matrix in Section 4 (not Standard DH).
- You included the offsets $\theta_{0,3}=-\pi/2$ and $\theta_{0,6}=+\pi$.
- You included the joint sign vector $s=[-1,1,1,-1,1,-1]$.

---

## 8) Validation Plan (FK)
### 8.1 Test configurations
Choose 5–10 joint configurations $q^{(k)}$ including:
- all-zeros,
- moderate random joint angles,
- near-limit cases (but still safe).

For each test:
1. Compute $T^{(k)}_{\text{FK}} = {}^{0}T_6(q^{(k)})$.
2. Obtain a reference tool pose $T^{(k)}_{\text{ref}}$ from your chosen reference tool (e.g., RoboDK pose readout, a trusted simulator, or RobotKinematicsCatalogue FK script).

### 8.2 Comparison metrics
Let $p_{\text{FK}}$, $R_{\text{FK}}$ be from your FK; $p_{\text{ref}}$, $R_{\text{ref}}$ from reference.

- Position error:
$$
e_p = \|p_{\text{FK}}-p_{\text{ref}}\|.
$$

- Orientation error via relative rotation:
$$
R_{\text{err}} = R_{\text{FK}}^T R_{\text{ref}},
\qquad
e_R = \cos^{-1}\left(\frac{\mathrm{tr}(R_{\text{err}})-1}{2}\right).
$$

Report $e_p$ (mm) and $e_R$ (rad or deg) in a small table.

---

## 9) Output of Task A
At the end of Task A you should have:
- A reproducible FK implementation producing ${}^{0}T_6(q)$
- A short validation table and/or screenshots demonstrating agreement with the reference
- This derivation document showing the exact transform chain and parameters
