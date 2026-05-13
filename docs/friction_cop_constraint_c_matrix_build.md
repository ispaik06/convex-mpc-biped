# Friction Pyramid, CoP Constraint, and Sparse Constraint Matrix Build

This document explains the **contact wrench inequality constraints** used in the Convex MPC formulation for the biped controller, with special focus on the **frame issue**: the MPC input wrench is expressed in the **world frame**, while the friction pyramid and CoP support polygon are naturally defined in the **foot contact frame**.

The correct implementation is:

$$
C_W = C_F
\begin{bmatrix}
R_{WF}^{T} & 0 \\
0 & R_{WF}^{T}
\end{bmatrix}
$$

where $C_F$ is the foot-frame contact constraint matrix and $C_W$ is the matrix applied directly to the world-frame MPC input.

---

## 1. Problem Setup

For each foot, the MPC input is assumed to be a **world-frame contact wrench**:

$$
u_i =
\begin{bmatrix}
F_{x,W} \\
F_{y,W} \\
F_{z,W} \\
M_{x,W} \\
M_{y,W} \\
M_{z,W}
\end{bmatrix}
=
\begin{bmatrix}
F_W \\
M_W
\end{bmatrix}
\in \mathbb{R}^{6}
$$

where:

- $F_W$ is the ground reaction force expressed in the world frame.
- $M_W$ is the contact moment expressed in the world frame.
- The moment must be taken about the **foot contact frame origin**, not about the torso COM.

For two feet, the per-step MPC input is ordered as:

$$
u_k =
\begin{bmatrix}
F_{Lx,W} \\
F_{Ly,W} \\
F_{Lz,W} \\
F_{Rx,W} \\
F_{Ry,W} \\
F_{Rz,W} \\
M_{Lx,W} \\
M_{Ly,W} \\
M_{Lz,W} \\
M_{Rx,W} \\
M_{Ry,W} \\
M_{Rz,W}
\end{bmatrix}
\in \mathbb{R}^{12}
$$

or, in scalar column order:

$$
 u_k =
 [F_{Lx}, F_{Ly}, F_{Lz}, F_{Rx}, F_{Ry}, F_{Rz}, M_{Lx}, M_{Ly}, M_{Lz}, M_{Rx}, M_{Ry}, M_{Rz}]^{T}_{W}
$$

Therefore the local-to-global column maps are:

$$
\text{leftMap} = [0,1,2,6,7,8]
$$

$$
\text{rightMap} = [3,4,5,9,10,11]
$$

---

## 2. Frames and Rotation Convention

Let $F$ denote the **foot contact frame**, and $W$ denote the **world frame**.

Assume the stance foot yaw angle in the world frame is known:

$$
\psi_f
$$

For a yaw-only foot frame, the rotation from foot frame to world frame is:

$$
R_{WF} = R_z(\psi_f)
=
\begin{bmatrix}
\cos\psi_f & -\sin\psi_f & 0 \\
\sin\psi_f & \cos\psi_f & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

Define:

$$
c = \cos\psi_f, \qquad s = \sin\psi_f
$$

Then:

$$
R_{WF} =
\begin{bmatrix}
c & -s & 0 \\
s & c & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

A vector expressed in the foot frame maps to the world frame by:

$$
v_W = R_{WF}v_F
$$

Therefore a world-frame vector maps into the foot frame by:

$$
v_F = R_{WF}^{T}v_W
$$

Since force and moment are both vectors, the wrench transform is:

$$
\begin{bmatrix}
F_F \\
M_F
\end{bmatrix}
=
\begin{bmatrix}
R_{WF}^{T} & 0 \\
0 & R_{WF}^{T}
\end{bmatrix}
\begin{bmatrix}
F_W \\
M_W
\end{bmatrix}
$$

Explicitly:

$$
F_{x,F} = cF_{x,W} + sF_{y,W}
$$

$$
F_{y,F} = -sF_{x,W} + cF_{y,W}
$$

$$
F_{z,F} = F_{z,W}
$$

and:

$$
M_{x,F} = cM_{x,W} + sM_{y,W}
$$

$$
M_{y,F} = -sM_{x,W} + cM_{y,W}
$$

$$
M_{z,F} = M_{z,W}
$$

> [!WARNING]
> Do **not** apply the CoP constraint directly to world-frame $M_x$ and $M_y$ unless the foot yaw is exactly zero. The foot support polygon is attached to the foot frame, not the world frame and not necessarily the torso/body frame.

---

## 3. Local Foot-Frame Contact Constraints

The local foot-frame wrench is:

$$
 w_F = [F_{x,F}, F_{y,F}, F_{z,F}, M_{x,F}, M_{y,F}, M_{z,F}]^{T}
$$

The controller uses 12 inequality rows per contact foot:

1. Four friction pyramid rows
2. Two normal force bound rows
3. Four CoP support polygon rows
4. Two torsional friction rows

These can be written compactly as:

$$
C_F w_F \le b_F
$$

where $C_F \in \mathbb{R}^{12\times 6}$.

---

## 4. Friction Pyramid Derivation

### 4.1 Physical Friction Cone

The Coulomb friction cone is:

$$
\sqrt{F_{x,F}^{2} + F_{y,F}^{2}} \le \mu F_{z,F}
$$

with:

$$
F_{z,F} \ge 0
$$

This circular cone is nonlinear. For Convex MPC, it is approximated as a friction pyramid:

$$
|F_{x,F}| \le \mu F_{z,F}
$$

$$
|F_{y,F}| \le \mu F_{z,F}
$$

This produces four linear inequalities:

$$
F_{x,F} - \mu F_{z,F} \le 0
$$

$$
-F_{x,F} - \mu F_{z,F} \le 0
$$

$$
F_{y,F} - \mu F_{z,F} \le 0
$$

$$
-F_{y,F} - \mu F_{z,F} \le 0
$$

### 4.2 Foot-Frame Friction Matrix Rows

For:

$$
w_F =
\begin{bmatrix}
F_{x,F} \\
F_{y,F} \\
F_{z,F} \\
M_{x,F} \\
M_{y,F} \\
M_{z,F}
\end{bmatrix}
$$

we get:

$$
C_{\text{fric},F} =
\begin{bmatrix}
1 & 0 & -\mu & 0 & 0 & 0 \\
-1 & 0 & -\mu & 0 & 0 & 0 \\
0 & 1 & -\mu & 0 & 0 & 0 \\
0 & -1 & -\mu & 0 & 0 & 0
\end{bmatrix}
$$

with bound:

$$
b_{\text{fric}} =
 [0, 0, 0, 0]^{T}
$$

### 4.3 World-Frame Friction Rows

Using:

$$
F_{x,F}=cF_{x,W}+sF_{y,W}
$$

$$
F_{y,F}=-sF_{x,W}+cF_{y,W}
$$

$$
F_{z,F}=F_{z,W}
$$

we get:

$$
F_{x,F} - \mu F_z \le 0
\Rightarrow
cF_{x,W}+sF_{y,W}-\mu F_{z,W}\le 0
$$

$$
-F_{x,F} - \mu F_z \le 0
\Rightarrow
-cF_{x,W}-sF_{y,W}-\mu F_{z,W}\le 0
$$

$$
F_{y,F} - \mu F_z \le 0
\Rightarrow
-sF_{x,W}+cF_{y,W}-\mu F_{z,W}\le 0
$$

$$
-F_{y,F} - \mu F_z \le 0
\Rightarrow
sF_{x,W}-cF_{y,W}-\mu F_{z,W}\le 0
$$

Therefore:

$$
C_{\text{fric},W}=
\begin{bmatrix}
c & s & -\mu & 0 & 0 & 0 \\
-c & -s & -\mu & 0 & 0 & 0 \\
-s & c & -\mu & 0 & 0 & 0 \\
s & -c & -\mu & 0 & 0 & 0
\end{bmatrix}
$$

> [!TIP]
> This is why the sparse pattern for each friction row must include **Fx, Fy, and Fz**. A yaw-rotated friction row is no longer only `{Fx, Fz}` or `{Fy, Fz}`.

---

## 5. Normal Force Bounds

The normal force is bounded by:

$$
F_{\min} \le F_{z,F} \le F_{\max}
$$

Because yaw rotation does not change $z$:

$$
F_{z,F}=F_{z,W}
$$

The upper bound is:

$$
F_{z,W} \le F_{\max}
$$

The lower bound is:

$$
F_{z,W} \ge F_{\min}
$$

which is written in upper-bound inequality form as:

$$
-F_{z,W} \le -F_{\min}
$$

Therefore the normal force rows are:

$$
C_{\text{normal},W} =
\begin{bmatrix}
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & -1 & 0 & 0 & 0
\end{bmatrix}
$$

with bound:

$$
b_{\text{normal}}=
 [F_{\max}, -F_{\min}]^{T}
$$

> [!NOTE]
> In the current code, the dense template uses rows `+Fz` and `-Fz`, and the actual bound vector is updated with `normalForceMax` and scaled `normalForceMin` depending on stance/contact transition logic.

---

## 6. CoP Constraint Derivation

### 6.1 CoP Definition

The CoP is defined on the foot sole plane. With foot-frame contact wrench:

$$
 w_F = [F_{x,F}, F_{y,F}, F_{z,F}, M_{x,F}, M_{y,F}, M_{z,F}]^{T}
$$

The CoP coordinates are:

$$
x_{cop,F} = -\frac{M_{y,F}}{F_{z,F}}
$$

$$
y_{cop,F} = \frac{M_{x,F}}{F_{z,F}}
$$

Assume:

$$
F_{z,F} > 0
$$

Let:

- $a$ = foot half length, along local $x_F$
- $b$ = foot half width, along local $y_F$

The rectangular support polygon is:

$$
-a \le x_{cop,F} \le a
$$

$$
-b \le y_{cop,F} \le b
$$

---

### 6.2 Length-Direction CoP Constraint

The $x$-direction CoP condition is:

$$
-a \le -\frac{M_{y,F}}{F_{z,F}} \le a
$$

Multiplying by $F_{z,F}>0$:

$$
-aF_{z,F} \le -M_{y,F} \le aF_{z,F}
$$

This gives:

$$
-M_{y,F} \le aF_{z,F}
$$

and:

$$
-M_{y,F} \ge -aF_{z,F}
$$

The second inequality is equivalent to:

$$
M_{y,F} \le aF_{z,F}
$$

Therefore:

$$
M_{y,F} - aF_{z,F} \le 0
$$

$$
-M_{y,F} - aF_{z,F} \le 0
$$

---

### 6.3 Width-Direction CoP Constraint

The $y$-direction CoP condition is:

$$
-b \le \frac{M_{x,F}}{F_{z,F}} \le b
$$

Multiplying by $F_{z,F}>0$:

$$
-bF_{z,F} \le M_{x,F} \le bF_{z,F}
$$

Therefore:

$$
M_{x,F} - bF_{z,F} \le 0
$$

$$
-M_{x,F} - bF_{z,F} \le 0
$$

---

### 6.4 Foot-Frame CoP Matrix Rows

Using:

$$
w_F =
\begin{bmatrix}
F_{x,F} \\
F_{y,F} \\
F_{z,F} \\
M_{x,F} \\
M_{y,F} \\
M_{z,F}
\end{bmatrix}
$$

The CoP rows are:

$$
C_{\text{cop},F}=
\begin{bmatrix}
0 & 0 & -b & 1 & 0 & 0 \\
0 & 0 & -b & -1 & 0 & 0 \\
0 & 0 & -a & 0 & 1 & 0 \\
0 & 0 & -a & 0 & -1 & 0
\end{bmatrix}
$$

where rows 1--2 constrain $y_{cop}$ through $M_x$, and rows 3--4 constrain $x_{cop}$ through $M_y$.

> [!NOTE]
> The code currently uses `footHalfWidth` for the $M_x$ rows and `footHalfLength` for the $M_y$ rows. This is correct if:
>
> - local $x_F$ is the foot length direction,
> - local $y_F$ is the foot width direction,
> - $x_{cop}=-M_y/F_z$,
> - $y_{cop}=M_x/F_z$.

---

### 6.5 World-Frame CoP Rows

Using:

$$
M_{x,F}=cM_{x,W}+sM_{y,W}
$$

$$
M_{y,F}=-sM_{x,W}+cM_{y,W}
$$

and:

$$
F_{z,F}=F_{z,W}
$$

#### Width-direction rows

$$
M_{x,F} - bF_z \le 0
$$

becomes:

$$
(cM_{x,W}+sM_{y,W}) - bF_{z,W}\le 0
$$

so the row is:

$$
\begin{bmatrix}
0 & 0 & -b & c & s & 0
\end{bmatrix}
$$

Next:

$$
-M_{x,F} - bF_z \le 0
$$

becomes:

$$
-(cM_{x,W}+sM_{y,W}) - bF_{z,W}\le 0
$$

so the row is:

$$
\begin{bmatrix}
0 & 0 & -b & -c & -s & 0
\end{bmatrix}
$$

#### Length-direction rows

$$
M_{y,F} - aF_z \le 0
$$

becomes:

$$
(-sM_{x,W}+cM_{y,W}) - aF_{z,W}\le 0
$$

so the row is:

$$
\begin{bmatrix}
0 & 0 & -a & -s & c & 0
\end{bmatrix}
$$

Next:

$$
-M_{y,F} - aF_z \le 0
$$

becomes:

$$
-(-sM_{x,W}+cM_{y,W}) - aF_{z,W}\le 0
$$

so the row is:

$$
\begin{bmatrix}
0 & 0 & -a & s & -c & 0
\end{bmatrix}
$$

Therefore:

$$
C_{\text{cop},W}=
\begin{bmatrix}
0 & 0 & -b & c & s & 0 \\
0 & 0 & -b & -c & -s & 0 \\
0 & 0 & -a & -s & c & 0 \\
0 & 0 & -a & s & -c & 0
\end{bmatrix}
$$

with zero upper bound:

$$
b_{\text{cop}}=0
$$

> [!IMPORTANT]
> Every CoP row after yaw rotation can touch **Fz, Mx, and My**. Therefore the sparse pattern must include local columns `{2, 3, 4}` for all four CoP rows.

---

## 7. Torsional Friction Constraint

The torsional friction constraint bounds the free yaw moment:

$$
|M_{z,F}| \le \mu_t F_{z,F}
$$

where:

$$
\mu_t = \text{torsionalFrictionScale}\cdot \mu
$$

This gives:

$$
M_{z,F} - \mu_t F_{z,F} \le 0
$$

$$
-M_{z,F} - \mu_t F_{z,F} \le 0
$$

Yaw-only rotation keeps:

$$
M_{z,F}=M_{z,W}, \qquad F_{z,F}=F_{z,W}
$$

Therefore:

$$
C_{\text{torsion},W}=
\begin{bmatrix}
0 & 0 & -\mu_t & 0 & 0 & 1 \\
0 & 0 & -\mu_t & 0 & 0 & -1
\end{bmatrix}
$$

with zero upper bound.

---

## 8. Full One-Foot Constraint Matrix

### 8.1 Foot-Frame Template

The complete local foot-frame template is:

$$
C_F =
\begin{bmatrix}
1 & 0 & -\mu & 0 & 0 & 0 \\
-1 & 0 & -\mu & 0 & 0 & 0 \\
0 & 1 & -\mu & 0 & 0 & 0 \\
0 & -1 & -\mu & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & -1 & 0 & 0 & 0 \\
0 & 0 & -b & 1 & 0 & 0 \\
0 & 0 & -b & -1 & 0 & 0 \\
0 & 0 & -a & 0 & 1 & 0 \\
0 & 0 & -a & 0 & -1 & 0 \\
0 & 0 & -\mu_t & 0 & 0 & 1 \\
0 & 0 & -\mu_t & 0 & 0 & -1
\end{bmatrix}
$$

where:

$$
a=\text{footHalfLength}, \qquad b=\text{footHalfWidth}, \qquad \mu_t=\text{torsionalFrictionScale}\cdot\mu
$$

The corresponding upper bound is:

$$
b_F=
[0, 0, 0, 0, F_{\max}, -F_{\min}, 0, 0, 0, 0, 0, 0]^{T}
$$

---

### 8.2 World-Frame One-Foot Matrix

The final one-foot matrix applied to world-frame input is:

$$
C_W = C_F
\begin{bmatrix}
R_{WF}^{T} & 0 \\
0 & R_{WF}^{T}
\end{bmatrix}
$$

For yaw-only rotation, explicitly:

$$
C_W =
\begin{bmatrix}
c & s & -\mu & 0 & 0 & 0 \\
-c & -s & -\mu & 0 & 0 & 0 \\
-s & c & -\mu & 0 & 0 & 0 \\
s & -c & -\mu & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & -1 & 0 & 0 & 0 \\
0 & 0 & -b & c & s & 0 \\
0 & 0 & -b & -c & -s & 0 \\
0 & 0 & -a & -s & c & 0 \\
0 & 0 & -a & s & -c & 0 \\
0 & 0 & -\mu_t & 0 & 0 & 1 \\
0 & 0 & -\mu_t & 0 & 0 & -1
\end{bmatrix}
$$

> [!TIP]
> This is the exact matrix that should be represented by the dense `C_left` or `C_right` foot block after yaw rotation.

---

## 9. Two-Foot Per-Step Inequality Matrix

For each MPC horizon step $k$, the inequality matrix has 24 rows:

$$
C_k u_k \le b_k
$$

where:

$$
C_k \in \mathbb{R}^{24\times 12}, \qquad u_k\in\mathbb{R}^{12}
$$

The first 12 rows are left-foot constraints, and the next 12 rows are right-foot constraints:

$$
C_k = [C_{L,k}; C_{R,k}]
$$

where each block is embedded into the 12D input vector.

If:

$$
C_{L,W}\in\mathbb{R}^{12\times 6}
$$

then it is placed into columns:

$$
[0,1,2,6,7,8]
$$

If:

$$
C_{R,W}\in\mathbb{R}^{12\times 6}
$$

then it is placed into columns:

$$
[3,4,5,9,10,11]
$$

The global structure is:

$$
C_k = [C_{L,F}T_{F_LW} \;\text{embedded into left columns}; C_{R,F}T_{F_RW} \;\text{embedded into right columns}]
$$

where:

$$
T_{F_iW}=\mathrm{blkdiag}(R_{WF_i}^{T}, R_{WF_i}^{T})
$$

Each foot should use its own foot yaw:

$$
\psi_L \ne \psi_R \quad \text{in general}
$$

---

## 10. Horizon-Level Constraint Matrix

For a horizon of $N$ steps, the full inequality matrix is block diagonal:

$$
C = \mathrm{blkdiag}(C_0, C_1, \ldots, C_{N-1})
$$

where:

$$
C\in\mathbb{R}^{24N\times 12N}
$$

The stacked input vector is:

$$
U = [u_0, u_1, \ldots, u_{N-1}]^{T}
\in\mathbb{R}^{12N}
$$

The stacked inequality is:

$$
C U \le b
$$

---

## 11. Equality Constraint Matrix `D`

In addition to the inequality matrix `C`, the controller also builds an equality matrix `D`:

$$
D U = d
$$

In this controller, `D` is mainly used for:

1. Swing-foot force zero constraints
2. Swing-foot torque zero constraints
3. Optional no-roll-moment stance constraint

For each horizon step:

$$
D_k\in\mathbb{R}^{12\times 12}
$$

The input is:

$$
u_k = [F_L, F_R, M_L, M_R]^{T}
$$

If a foot is in swing, its selection matrix is identity, forcing the corresponding wrench components to zero.

For example, if the left foot is in swing:

$$
F_L=0, \qquad M_L=0
$$

If it is in stance, the corresponding rows are zero unless an additional contact wrench model is active.

### Optional No-Roll-Moment Constraint

If the contact wrench model disables foot roll moment, the constraint should be expressed with the foot local x-axis in the world frame:

$$
\hat{x}_{F,W}^{T}M_W=0
$$

This is better than forcing world $M_x=0$, because foot roll is not necessarily aligned with the world $x$ axis when the foot yaw changes.

> [!IMPORTANT]
> The no-roll-moment constraint is also a frame issue. The correct row uses the foot local x-axis expressed in world coordinates, not the body yaw and not the world x-axis by default.

---

## 12. Code Implementation

The relevant implementation is split across three places:

- `My_Controller/src/My_Controller.cpp`: resolves the per-foot yaw that gets used for the constraint rotation and passes it into the gait scheduler.
- `My_Controller/src/GaitScheduler.cpp`: builds `C_unit`, rotates each foot block with `fillYawRotatedFootConstraintBlock`, and fills the horizon matrices `C`, `D`, and `C_bound`.
- `My_Controller/src/ConvexMPC.cpp`: copies the already-built dense `C` and `D` matrices into the OSQP sparse structure through `buildConstraintMatrix()`.

## 12.1 `GaitScheduler.cpp`: Build the Local Template

The local template should be built in foot-frame coordinates:

```cpp
C_unit <<
    1,  0, -mu, 0,  0,  0,
   -1,  0, -mu, 0,  0,  0,
    0,  1, -mu, 0,  0,  0,
    0, -1, -mu, 0,  0,  0,
    0,  0,  1,  0,  0,  0,
    0,  0, -1,  0,  0,  0,
    0,  0, -footHalfWidth,   1,  0,  0,
    0,  0, -footHalfWidth,  -1,  0,  0,
    0,  0, -footHalfLength,  0,  1,  0,
    0,  0, -footHalfLength,  0, -1,  0,
    0,  0, -mu_t, 0,  0,  1,
    0,  0, -mu_t, 0,  0, -1;
```

where:

```cpp
const double mu   = mpc.frictionCoefficient;
const double mu_t = mpc.torsionalFrictionScale * mpc.frictionCoefficient;
```

---

## 12.2 `GaitScheduler.cpp`: Rotate Local Rows into World-Frame Input Rows

Given one local foot row:

```cpp
[fx, fy, fz, mx, my, mz]
```

we want the row that acts directly on:

```cpp
[Fx_W, Fy_W, Fz_W, Mx_W, My_W, Mz_W]
```

Using:

$$
C_W = C_F
\begin{bmatrix}
R_{WF}^{T} & 0 \\
0 & R_{WF}^{T}
\end{bmatrix}
$$

For yaw-only rotation, the row coefficients become:

$$F_x = c\,fx - s\,fy$$
$$F_y = s\,fx + c\,fy$$
$$F_z = fz$$
$$M_x = c\,mx - s\,my$$
$$M_y = s\,mx + c\,my$$
$$M_z = mz$$

Implementation:

```cpp
void fillYawRotatedFootConstraintBlock(
    const Eigen::Matrix<double, 12, 6>& localBlock,
    const double yaw_W,
    Eigen::Matrix<double, 12, 12>& worldBlock,
    const int forceColOffset,
    const int momentColOffset) {

    const double c = std::cos(yaw_W);
    const double s = std::sin(yaw_W);

    worldBlock.setZero();

    for (int row = 0; row < 12; ++row) {
        const double fx = localBlock(row, 0);
        const double fy = localBlock(row, 1);
        const double fz = localBlock(row, 2);
        const double mx = localBlock(row, 3);
        const double my = localBlock(row, 4);
        const double mz = localBlock(row, 5);

        worldBlock(row, forceColOffset + 0) = c * fx - s * fy;
        worldBlock(row, forceColOffset + 1) = s * fx + c * fy;
        worldBlock(row, forceColOffset + 2) = fz;

        worldBlock(row, momentColOffset + 0) = c * mx - s * my;
        worldBlock(row, momentColOffset + 1) = s * mx + c * my;
        worldBlock(row, momentColOffset + 2) = mz;
    }
}
```

For the left foot:

```cpp
fillYawRotatedFootConstraintBlock(
    C_unit,
    resolvedLeftFootYaw_W,
    C_left,
    0,   // left force starts at columns 0..2
    6);  // left moment starts at columns 6..8
```

For the right foot:

```cpp
fillYawRotatedFootConstraintBlock(
    C_unit,
    resolvedRightFootYaw_W,
    C_right,
    3,   // right force starts at columns 3..5
    9);  // right moment starts at columns 9..11
```

---

## 12.3 `ConvexMPC.cpp`: Sparse Pattern Must Match the Yaw-Rotated Rows

The dense `C` matrix can contain new nonzero terms after yaw rotation. The OSQP sparse matrix must reserve slots for these terms.

The required row-level nonzero pattern in local 6D foot wrench coordinates is:

```cpp
const std::array<std::vector<int>, 12> cUnitCols = {{
    // Friction pyramid in foot-local frame, rotated into world frame.
    // Fx_F = c Fx_W + s Fy_W
    // Fy_F = -s Fx_W + c Fy_W
    // Therefore each tangential friction row can touch Fx_W, Fy_W, Fz_W.
    {0, 1, 2}, // +Fx_F - mu Fz <= 0
    {0, 1, 2}, // -Fx_F - mu Fz <= 0
    {0, 1, 2}, // +Fy_F - mu Fz <= 0
    {0, 1, 2}, // -Fy_F - mu Fz <= 0

    // Normal force bounds.
    // Yaw rotation does not change z.
    {2},       // +Fz <= Fz_max
    {2},       // -Fz <= -Fz_min

    // CoP bounds in foot-local frame, rotated into world frame.
    // Mx_F = c Mx_W + s My_W
    // My_F = -s Mx_W + c My_W
    // Therefore each CoP row can touch Fz_W, Mx_W, My_W.
    {2, 3, 4}, // +Mx_F - footHalfWidth  Fz <= 0
    {2, 3, 4}, // -Mx_F - footHalfWidth  Fz <= 0
    {2, 3, 4}, // +My_F - footHalfLength Fz <= 0
    {2, 3, 4}, // -My_F - footHalfLength Fz <= 0

    // Torsional friction.
    // Yaw-only rotation keeps Mz unchanged.
    {2, 5},    // +Mz - mu_t Fz <= 0
    {2, 5},    // -Mz - mu_t Fz <= 0
}};
```

> [!WARNING]
> The older pattern
>
> ```cpp
> {0, 2}, {0, 2}, {1, 2}, {1, 2},
> {2, 3}, {2, 3}, {2, 4}, {2, 4}
> ```
>
> is valid only when foot yaw is zero. It is not sufficient after yaw rotation.

The copy into the solver happens in `ConvexMPC::buildConstraintMatrix(const DMat<double>& C, const DMat<double>& D)`, which forwards the dense matrices into `fillConstraintValues(...)` without changing the frame convention.

---

## 13. Sparse Matrix Optimization

## 13.1 Why Not Use a Fully Dense Constraint Pattern?

For one horizon step:

- `C_k` has shape $24\times 12$.
- A fully dense pattern would have:

$$
24\times 12 = 288
$$

potential nonzero entries per step.

But the true contact constraints are very sparse. With the yaw-rotated superset pattern:

### One foot

Friction rows:

$$
4\times 3 = 12
$$

Normal rows:

$$
2\times 1 = 2
$$

CoP rows:

$$
4\times 3 = 12
$$

Torsional rows:

$$
2\times 2 = 4
$$

Total per foot:

$$
12+2+12+4=30
$$

Two feet:

$$
30\times 2=60
$$

So the optimized yaw-safe inequality pattern uses:

$$
60 \quad \text{C entries per horizon step}
$$

instead of:

$$
288 \quad \text{dense C entries per horizon step}
$$

---

## 13.2 Equality Matrix Pattern Size

The equality matrix `D_k` uses 12 rows. The optimized pattern reserves:

- Force swing-zero diagonal rows: 6 entries
- Torque swing-zero diagonal rows: 4 entries for normal swing-zero torque rows, plus additional slots for no-roll rows
- No-roll moment rows: 3 torque columns for left row 6 and 3 torque columns for right row 9

In the current row-level pattern, this is treated as:

$$
16 \quad \text{D entries per horizon step}
$$

Therefore the total optimized sparse constraint pattern should reserve:

$$
60 + 16 = 76
$$

entries per horizon step.

Implementation:

```cpp
// Yaw-rotated GaitScheduler pattern:
//
// Per step:
// C: 60 entries
// D: 16 entries
// total: 76 entries
triplets.reserve(static_cast<std::size_t>(steps) * 76);
```

---

## 13.3 Why This Is an Optimization

OSQP expects the sparse matrix structure to remain fixed after setup. Updating only numeric values is much faster than rebuilding the sparse structure every MPC solve.

The optimized design is:

1. Build the sparse pattern once in the `ConvexMPC` constructor.
2. Reuse the same sparse matrix structure every MPC iteration.
3. Update only the numeric values of existing sparse entries.
4. Include a **yaw-safe superset** of possible nonzero entries.

This avoids:

- Reallocating sparse matrices every solve
- Rebuilding OSQP matrix structure every solve
- Accidentally dropping yaw-induced nonzero terms
- Using a fully dense and unnecessarily expensive constraint matrix

> [!IMPORTANT]
> The goal is not to create the smallest possible pattern for each foot yaw. The goal is to create a **fixed sparse superset pattern** that is valid for all possible foot yaw values.

---

## 14. Correct `makeRowLevelConstraintPattern()` Patch

The required patch in `My_Controller/src/ConvexMPC.cpp` is:

```diff
-// Current GaitScheduler pattern:
+// Yaw-rotated GaitScheduler pattern:
 //
 // Per step:
-// C: 44 entries
+// C: 60 entries
 // D: 16 entries
-// total: 60 entries
-triplets.reserve(static_cast<std::size_t>(steps) * 60);
+// total: 76 entries
+triplets.reserve(static_cast<std::size_t>(steps) * 76);
```

and:

```diff
 const std::array<std::vector<int>, 12> cUnitCols = {{
-    {0, 2}, // +Fx - mu Fz <= 0
-    {0, 2}, // -Fx - mu Fz <= 0
-    {1, 2}, // +Fy - mu Fz <= 0
-    {1, 2}, // -Fy - mu Fz <= 0
+    {0, 1, 2}, // +Fx_F - mu Fz <= 0
+    {0, 1, 2}, // -Fx_F - mu Fz <= 0
+    {0, 1, 2}, // +Fy_F - mu Fz <= 0
+    {0, 1, 2}, // -Fy_F - mu Fz <= 0
     {2},       // +Fz <= Fz_max
     {2},       // -Fz <= -Fz_min
-    {2, 3}, // +Mx - w Fz <= 0
-    {2, 3}, // -Mx - w Fz <= 0
-    {2, 4}, // +My - l Fz <= 0
-    {2, 4}, // -My - l Fz <= 0
-    {2, 5}, // +Mz - mu_t Fz <= 0
-    {2, 5}, // -Mz - mu_t Fz <= 0
+    {2, 3, 4}, // +Mx_F - footHalfWidth  Fz <= 0
+    {2, 3, 4}, // -Mx_F - footHalfWidth  Fz <= 0
+    {2, 3, 4}, // +My_F - footHalfLength Fz <= 0
+    {2, 3, 4}, // -My_F - footHalfLength Fz <= 0
+    {2, 5},    // +Mz - mu_t Fz <= 0
+    {2, 5},    // -Mz - mu_t Fz <= 0
 }};
```

---

## 15. Debug Safety Check

The current `fillConstraintValues()` design should keep the debug check:

```cpp
#ifndef NDEBUG
// If C or D contains a nonzero value outside the sparse pattern,
// throw an error instead of silently dropping it.
#endif
```

This is important because the sparse pattern is only a **structural promise**. If the dense `C` matrix later introduces a nonzero coefficient outside the reserved sparse slots, OSQP will not see that coefficient unless a sparse slot exists.

A useful debug error looks like:

```text
Constraint sparse pattern missing C entry at row=..., col=..., value=...
```

If this happens, it means one of the constraint rows is producing a nonzero term that the sparse pattern did not reserve.

> [!TIP]
> Always test this in a debug build with a nonzero foot yaw. If the sparse pattern is incomplete, yaw-rotated friction or CoP rows are the easiest way to expose the bug.

---

## 16. Verification Checklist

### 16.1 Friction Rows

For nonzero foot yaw, the first four rows of a one-foot block should be:

$$
\begin{bmatrix}
c & s & -\mu & 0 & 0 & 0 \\
-c & -s & -\mu & 0 & 0 & 0 \\
-s & c & -\mu & 0 & 0 & 0 \\
s & -c & -\mu & 0 & 0 & 0
\end{bmatrix}
$$

If row 0 only touches `Fx` and `Fz`, the sparse pattern is wrong.

---

### 16.2 CoP Rows

The CoP rows should be:

$$
\begin{bmatrix}
0 & 0 & -b & c & s & 0 \\
0 & 0 & -b & -c & -s & 0 \\
0 & 0 & -a & -s & c & 0 \\
0 & 0 & -a & s & -c & 0
\end{bmatrix}
$$

If a CoP row only touches `Fz` and one of `Mx` or `My`, the sparse pattern is wrong for nonzero yaw.

---

### 16.3 Normal Force Rows

The normal force rows should remain:

$$
\begin{bmatrix}
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & -1 & 0 & 0 & 0
\end{bmatrix}
$$

Yaw does not affect these rows.

---

### 16.4 Torsional Rows

The torsional rows should remain:

$$
\begin{bmatrix}
0 & 0 & -\mu_t & 0 & 0 & 1 \\
0 & 0 & -\mu_t & 0 & 0 & -1
\end{bmatrix}
$$

Yaw-only rotation does not affect $M_z$.

---

## 17. Moment Reference Point Warning

The CoP formula:

$$
x_{cop,F} = -\frac{M_{y,F}}{F_{z,F}},
\qquad
 y_{cop,F}=\frac{M_{x,F}}{F_{z,F}}
$$

is valid only if $M_F$ is the moment about the foot contact frame origin.

If the MPC moment is instead a net moment about the torso COM, then the CoP constraint cannot be applied directly.

The relation is:

$$
M_{COM,W} = r_{contact\rightarrow COM,W}\times F_W + M_{contact,W}
$$

Therefore:

$$
M_{contact,W} = M_{COM,W} - r_{contact\rightarrow COM,W}\times F_W
$$

> [!WARNING]
> Apply the CoP constraint to $M_{contact,W}$, not to a moment that already includes the lever-arm term $r\times F$ about the torso COM.

---

## 18. Final Implementation Summary

The correct flow is:

```text
1. Build C_unit in foot-local wrench coordinates.

2. For each foot, get stance/touchdown foot yaw:
      psi_L, psi_R

3. Rotate local C_unit rows into world-frame input rows:
      C_left  = C_unit * blockdiag(R_L^T, R_L^T)
      C_right = C_unit * blockdiag(R_R^T, R_R^T)

4. Embed C_left and C_right into the per-step 12D input block:
      left columns  = [0,1,2,6,7,8]
      right columns = [3,4,5,9,10,11]

5. Stack per-step C_k over the horizon:
      C = blockdiag(C_0, ..., C_{N-1})

6. Build a fixed OSQP sparse pattern once using the yaw-rotated superset:
      friction rows: {0,1,2}
      normal rows:   {2}
      CoP rows:      {2,3,4}
      torsion rows:  {2,5}

7. At runtime, update only sparse numeric values, not the structure.
```

The key correction is:

```cpp
// Friction rows need Fx, Fy, Fz after yaw rotation.
{0, 1, 2}

// CoP rows need Fz, Mx, My after yaw rotation.
{2, 3, 4}
```

This makes the friction pyramid and CoP constraints frame-consistent with world-frame MPC inputs while preserving an efficient fixed sparse matrix structure for OSQP.
