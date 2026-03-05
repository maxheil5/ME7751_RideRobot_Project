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

$$
{}^{0}T_6 =
\begin{bmatrix}
R_{06} & p_{06} \\
0\ 0\ 0 & 1
\end{bmatrix},
$$

where $R_{06}\in SO(3)$ and $p_{06}\in \mathbb{R}^3$ (mm).

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
\theta_i(q_i) = \theta_{0,i} + s_i\,q_i
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

Define shorthand $c_i=\cos(\theta_i)$, $s_i=\sin(\theta_i)$, $c_{\alpha i}=\cos(\alpha_i)$, and $s_{\alpha i}=\sin(\alpha_i)$.

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

> Important: This is Modified DH, not Standard DH. Do not swap conventions mid-derivation.

---

## 5) Individual Joint Transforms (${}^{i-1}T_i$)
Below are the six transforms with the KR500 parameters substituted.  
Let $\theta_i$ be defined in Section 3.2.

### 5.1 ${}^{0}T_1$  ($\alpha_1=0,\ a_1=0,\ d_1=1045$)

Since $\cos 0 = 1$ and $\sin 0 = 0$:

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

Use $\cos(-\pi/2)=0$ and $\sin(-\pi/2)=-1$:

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

Use $\cos(-\pi/2)=0$ and $\sin(-\pi/2)=-1$:

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

Use $\cos(\pi/2)=0$ and $\sin(\pi/2)=1$:

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

Use $\cos(-\pi/2)=0$ and $\sin(-\pi/2)=-1$:

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

### 6.0 Shorthand (for compact closed form)

Define:
$$
c1=\cos q_1,\ s1=\sin q_1,\quad
c4=\cos q_4,\ s4=\sin q_4,\quad
c5=\cos q_5,\ s5=\sin q_5.
$$

Since $\theta_6=\pi-q_6$ (from offsets/signs), define:
$$
c6=\cos(\pi-q_6)=-\cos q_6,\qquad s6=\sin(\pi-q_6)=\sin q_6.
$$

Also define the coupled shoulder–elbow angle:
$$
c_{23}=\cos(q_2+q_3),\qquad s_{23}=\sin(q_2+q_3).
$$

Define the “arm geometry” scalar (mm):
$$
B = 500 + 1300\cos q_2 + 1025\,c_{23} - 55\,s_{23}.
$$

---

### 6.1 Closed-form rotation $R_{06}(q)$

Let:
$$
R_{06}=
\begin{bmatrix}
R_{11} & R_{12} & R_{13}\\
R_{21} & R_{22} & R_{23}\\
R_{31} & R_{32} & R_{33}
\end{bmatrix}.
$$

Third column (also used in the position offset):
$$
\begin{aligned}
R_{13}&=-s5\,(s1\,s4 + s_{23}\,c1\,c4) + c5\,(c1\,c_{23}),\\
R_{23}&=-s5\,(-s1\,s_{23}\,c4 + s4\,c1) + c5\,(-s1\,c_{23}),\\
R_{33}&=-s5\,(c4\,c_{23}) + c5\,(-s_{23}).
\end{aligned}
$$

First column:
$$
\begin{aligned}
R_{11}&=c6\,c5\,(s1\,s4 + s_{23}\,c1\,c4) + c6\,s5\,(c1\,c_{23}) + s6\,(-s1\,c4 + s_{23}\,c1\,s4),\\
R_{21}&=c6\,c5\,(-s1\,s_{23}\,c4 + s4\,c1) + c6\,s5\,(-s1\,c_{23}) + s6\,(-s1\,s4\,s_{23} - c1\,c4),\\
R_{31}&=c6\,c5\,(c4\,c_{23}) + c6\,s5\,(-s_{23}) + s6\,(s4\,c_{23}).
\end{aligned}
$$

Second column:
$$
\begin{aligned}
R_{12}&=-s6\,c5\,(s1\,s4 + s_{23}\,c1\,c4) - s6\,s5\,(c1\,c_{23}) + c6\,(-s1\,c4 + s_{23}\,c1\,s4),\\
R_{22}&=-s6\,c5\,(-s1\,s_{23}\,c4 + s4\,c1) - s6\,s5\,(-s1\,c_{23}) + c6\,(-s1\,s4\,s_{23} - c1\,c4),\\
R_{32}&=-s6\,c5\,(c4\,c_{23}) - s6\,s5\,(-s_{23}) + c6\,(s4\,c_{23}).
\end{aligned}
$$

---

### 6.2 Closed-form position $p_{06}(q)$ (mm)

First, the origin of frame 4 expressed in the base frame:
$$
p_{04}=
\begin{bmatrix}
c1\,B\\
-s1\,B\\
1045 -1300\sin q_2 -1025\,s_{23} -55\,c_{23}
\end{bmatrix}.
$$

The last link has $a_6=0$ and $d_6=290$, so the end-effector origin is:
$$
p_{06} = p_{04} + 290
\begin{bmatrix}
R_{13}\\
R_{23}\\
R_{33}
\end{bmatrix}.
$$

Equivalently, component-wise:
$$
\begin{aligned}
p_x &= c1\,B + 290\,R_{13},\\
p_y &= -s1\,B + 290\,R_{23},\\
p_z &= 1045 -1300\sin q_2 -1025\,s_{23} -55\,c_{23} + 290\,R_{33}.
\end{aligned}
$$

---

### 6.3 Final closed-form homogeneous transform ${}^{0}T_6(q)$

$$
{}^{0}T_6(q)=
\begin{bmatrix}
R_{11} & R_{12} & R_{13} & p_x\\
R_{21} & R_{22} & R_{23} & p_y\\
R_{31} & R_{32} & R_{33} & p_z\\
0 & 0 & 0 & 1
\end{bmatrix}.
$$

---

