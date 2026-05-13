# Yaw Wrapping Policy for Convex MPC Biped Control

> **Purpose**
> This document defines **where to use wrapped yaw**, **where to use unwrapped yaw**, and **what should be stored in `StateEstimate`** for a biped Convex MPC controller that receives **yaw-rate commands** rather than absolute yaw commands.
>
> The main goal is to remove failures around the \(\pm\pi\) boundary, especially during in-place turning near **180 degrees**.

---

## 0. Executive Summary

> [!IMPORTANT]
> The controller should treat yaw as a **continuous control state**, not as a bounded display angle.

Use this policy:

| Quantity / Module | Recommended yaw representation | Reason |
|---|---:|---|
| `StateEstimate` primary yaw | **Unwrapped** | Continuous MPC/control state |
| `StateEstimate` display yaw | **Wrapped** | Human-readable logs and UI |
| MPC initial state yaw | **Unwrapped** | Avoid \(+\pi \leftrightarrow -\pi\) jump in cost |
| MPC yaw reference over horizon | **Unwrapped** | Avoid discontinuity inside horizon |
| Yaw-rate estimation | **Wrap-safe delta** or gyro | Avoid fake huge yaw rate at \(\pm\pi\) |
| Yaw-rate command integration | **Unwrapped** | Continuous heading command |
| Rotation matrix `Rz(yaw)` | Either, but prefer unwrapped source | `sin/cos` are periodic |
| CoP/Friction constraint yaw | Either, because only `sin/cos` used | No branch issue if not differenced |
| Yaw error | **Wrapped difference** or branch-lifted target | Error must be shortest signed angle |
| Logs / viewer / UI | **Wrapped** | Easier to read |

Recommended `StateEstimate` fields:

```cpp
struct StateEstimate {
    Eigen::Quaterniond quat_WB;

    double roll_W;
    double pitch_W;

    // Yaw policy
    double yaw_W_wrapped;       // in [-pi, pi], from quaternion/R matrix
    double yaw_W_unwrapped;     // continuous yaw, primary for control/MPC
    double yawRate_W;           // wrap-safe yaw rate, rad/s

    // Optional but recommended if foot yaw is used beyond sin/cos
    double leftFootYaw_W_wrapped;
    double rightFootYaw_W_wrapped;
    double leftFootYaw_W_unwrapped;
    double rightFootYaw_W_unwrapped;
};
```

---

## 1. Why This Matters

Most yaw extraction functions return yaw in:

\[
\psi \in [-\pi, \pi]
\]

That representation is fine for display, but it is **not continuous**. During a smooth rotation, the measured yaw may jump like this:

```text
 3.10 rad ->  3.13 rad ->  3.14 rad -> -3.13 rad -> -3.10 rad
```

Physically, the robot turned smoothly. Numerically, however, the raw yaw difference becomes:

\[
\Delta \psi_{\text{raw}} = -3.13 - 3.14 \approx -6.27
\]

This creates a fake jump of almost:

\[
-2\pi
\]

If this raw difference enters the controller, the controller may generate a huge yaw rate, huge yaw error, wrong footstep yaw, wrong capture-point offset, or an unstable MPC solution.

> [!WARNING]
> A failure exactly around **180 degrees** is usually not caused by CoP/Friction constraints themselves.
> It is usually caused by **yaw wrap-around** in state, reference, yaw-rate calculation, or footstep planning.

---

## 2. Definitions

### 2.1 Wrapped Yaw

Wrapped yaw is bounded:

\[
\psi_{w} \in [-\pi, \pi]
\]

Use:

```cpp
double wrapToPi(double x) {
    return std::atan2(std::sin(x), std::cos(x));
}
```

Then:

\[
\psi_w = \mathrm{wrapToPi}(\psi)
\]

Wrapped yaw is good for:

- display
- logs
- UI
- external interfaces
- shortest-angle error calculation

It is **bad** as a continuous MPC state.

---

### 2.2 Unwrapped Yaw

Unwrapped yaw is continuous:

\[
\psi_u \in \mathbb{R}
\]

It can go beyond \(\pi\):

```text
0.0 -> 1.0 -> 2.0 -> 3.0 -> 3.2 -> 4.0 -> 5.0 -> 6.3 -> ...
```

Unwrapped yaw is good for:

- MPC state
- MPC reference trajectory
- integrating yaw-rate command
- yaw-rate calculation
- swing touchdown yaw planning
- any operation that assumes continuity

---

## 3. The Core Rule

> [!IMPORTANT]
> Use **unwrapped yaw** when yaw is treated as a **state**.
> Use **wrapped yaw** when yaw is treated as an **angle on a circle**.

More concretely:

### Use unwrapped yaw for subtraction across time

\[
\dot{\psi} \approx \frac{\psi_u[k] - \psi_u[k-1]}{\Delta t}
\]

### Use wrapped difference for shortest angular error

\[
e_{\psi} = \mathrm{wrapToPi}(\psi_{target} - \psi_{current})
\]

### Use either for rotation matrices

Because:

\[
\cos(\psi) = \cos(\psi + 2\pi n)
\]

\[
\sin(\psi) = \sin(\psi + 2\pi n)
\]

Therefore:

\[
R_z(\psi_w) = R_z(\psi_u)
\]

up to numerical precision.

---

## 4. Recommended StateEstimate Design

### 4.1 Required fields

```cpp
struct StateEstimate {
    Eigen::Quaterniond quat_WB;

    double roll_W;
    double pitch_W;

    double yaw_W_wrapped;       // [-pi, pi]
    double yaw_W_unwrapped;     // continuous yaw
    double yawRate_W;           // rad/s, wrap-safe
};
```

### 4.2 Why store both?

Storing only wrapped yaw is unsafe for control.

Storing only unwrapped yaw is inconvenient for logs, UI, and external commands.

So the best structure is:

> [!TIP]
> Store **both**.
> Treat `yaw_W_unwrapped` as the **primary control yaw**.
> Treat `yaw_W_wrapped` as the **display/interface yaw**.

---

## 5. Updating Unwrapped Yaw

### 5.1 Initialization

At startup:

```cpp
double yaw0_wrapped = extractYawFromQuaternion(quat_WB);

state.yaw_W_wrapped = yaw0_wrapped;
state.yaw_W_unwrapped = yaw0_wrapped;
state.yawRate_W = 0.0;
```

### 5.2 Runtime update

At every state-estimator update:

```cpp
double yawWrappedNow = extractYawFromQuaternion(quat_WB);

double dyaw = wrapToPi(yawWrappedNow - state.yaw_W_wrapped);

state.yaw_W_unwrapped += dyaw;
state.yawRate_W = dyaw / dt;
state.yaw_W_wrapped = yawWrappedNow;
```

Mathematically:

\[
\Delta \psi[k] = \mathrm{wrapToPi}(\psi_w[k] - \psi_w[k-1])
\]

\[
\psi_u[k] = \psi_u[k-1] + \Delta \psi[k]
\]

\[
\dot{\psi}[k] = \frac{\Delta \psi[k]}{\Delta t}
\]

This prevents the fake jump:

\[
3.14 \rightarrow -3.13
\]

from becoming:

\[
\Delta\psi \approx -6.27
\]

Instead it becomes:

\[
\Delta\psi \approx +0.01
\]

---

## 6. Since Commands Are Yaw-Rate Only

The user command is not an absolute heading target. It is:

\[
\dot{\psi}_{cmd}
\]

This simplifies the policy a lot.

> [!IMPORTANT]
> Do **not** wrap the yaw reference generated by yaw-rate integration.

### 6.1 Reference yaw generation

Given current unwrapped yaw:

\[
\psi_u[0]
\]

and yaw-rate command:

\[
\dot{\psi}_{cmd}
\]

The MPC horizon reference should be:

\[
\psi_{ref}[k] = \psi_u[0] + \dot{\psi}_{cmd} t_k
\]

where:

\[
t_k = k \Delta t_{mpc}
\]

Code:

```cpp
for (int k = 0; k < horizonLength; ++k) {
    double t = static_cast<double>(k + 1) * mpcDt;
    yawRef[k] = state.yaw_W_unwrapped + yawRateCmd * t;
}
```

Do **not** do this:

```cpp
// Bad: creates discontinuity around +/- pi
for (int k = 0; k < horizonLength; ++k) {
    yawRef[k] = wrapToPi(state.yaw_W_unwrapped + yawRateCmd * t);
}
```

### 6.2 Why not wrap reference yaw?

If the horizon crosses \(\pi\), wrapping produces:

```text
3.10, 3.13, -3.12, -3.09, ...
```

The MPC cost sees a huge discontinuity:

\[
3.13 \rightarrow -3.12
\]

which looks like a jump of about:

\[
-2\pi
\]

The correct unwrapped reference is:

```text
3.10, 3.13, 3.16, 3.19, ...
```

---

## 7. MPC State and Reference Policy

### 7.1 MPC initial state

Use:

```cpp
x0(yawIndex) = state.yaw_W_unwrapped;
```

Not:

```cpp
x0(yawIndex) = state.yaw_W_wrapped;
```

### 7.2 MPC reference state

Use:

```cpp
xRef[k](yawIndex) = state.yaw_W_unwrapped + yawRateCmd * t;
```

Not:

```cpp
xRef[k](yawIndex) = wrapToPi(state.yaw_W_unwrapped + yawRateCmd * t);
```

### 7.3 MPC yaw cost

If the MPC cost contains:

\[
(\psi - \psi_{ref})^2
\]

then both \(\psi\) and \(\psi_{ref}\) must live on the same continuous branch.

Therefore:

\[
\psi = \psi_u
\]

\[
\psi_{ref} = \psi_{u,ref}
\]

Then:

\[
\psi - \psi_{ref}
\]

is continuous.

---

## 8. Body-Aligned Frame and Rotation Matrices

Many parts of the controller use:

\[
R_z(\psi)
\]

For example:

\[
R_z(\psi) =
\begin{bmatrix}
\cos\psi & -\sin\psi & 0 \\
\sin\psi & \cos\psi & 0 \\
0 & 0 & 1
\end{bmatrix}
\]

Since `sin` and `cos` are periodic, both of these are acceptable:

```cpp
Rz(state.yaw_W_unwrapped);
Rz(state.yaw_W_wrapped);
```

Recommended convention:

> [!TIP]
> Use `yaw_W_unwrapped` as the source for all control calculations.
> Let `Rz()` call `std::sin` and `std::cos` directly.
> Only wrap for logging or explicit shortest-angle errors.

### 8.1 Numerical precision note

If the robot spins for a very long time, `yaw_W_unwrapped` could become very large. Extremely large arguments to `sin/cos` may lose precision.

For normal walking tests this is usually irrelevant.

If needed later, use a global yaw offset/rebasing method:

\[
\psi_u \leftarrow \psi_u - 2\pi n
\]

and apply the same offset consistently to all stored yaw references. Do **not** rebase only one part of the controller.

---

## 9. Friction Cone and CoP Constraint Yaw Policy

The friction cone and CoP constraints use the stance foot yaw to rotate foot-local constraints into the world-frame MPC input.

For one foot:

\[
u_W =
\begin{bmatrix}
F_{x,W} & F_{y,W} & F_{z,W} & M_{x,W} & M_{y,W} & M_{z,W}
\end{bmatrix}^T
\]

Foot yaw:

\[
\psi_f
\]

Define:

\[
c = \cos\psi_f
\]

\[
s = \sin\psi_f
\]

Then:

\[
F_{x,F} = cF_{x,W} + sF_{y,W}
\]

\[
F_{y,F} = -sF_{x,W} + cF_{y,W}
\]

\[
M_{x,F} = cM_{x,W} + sM_{y,W}
\]

\[
M_{y,F} = -sM_{x,W} + cM_{y,W}
\]

Because only \(\sin\) and \(\cos\) are used:

\[
\psi_{f,w}\quad \text{and}\quad \psi_{f,u}
\]

produce the same constraint matrix.

Therefore, for the constraint builder itself:

```cpp
double c = std::cos(footYaw);
double s = std::sin(footYaw);
```

`footYaw` may be either wrapped or unwrapped.

> [!IMPORTANT]
> The constraint builder is safe with wrapped yaw **only if it does not compute yaw differences, yaw rates, or interpolation**.

---

## 10. Foot Yaw Policy

Foot yaw has two possible uses:

1. **Orientation only**: used for `sin/cos` to rotate constraints.
2. **Continuous planning state**: used for touchdown yaw prediction, interpolation, or yaw error.

### 10.1 If foot yaw is only used for constraint rotation

Wrapped is acceptable:

```cpp
fillYawRotatedFootConstraintBlock(C_unit, footYawWrapped, C_leg, ...);
```

because:

\[
R_z(\psi_f) = R_z(\mathrm{wrapToPi}(\psi_f))
\]

### 10.2 If foot yaw is used for planning

Store both:

```cpp
state.leftFootYaw_W_wrapped;
state.leftFootYaw_W_unwrapped;
state.rightFootYaw_W_wrapped;
state.rightFootYaw_W_unwrapped;
```

Update them exactly like torso yaw:

```cpp
void updateUnwrappedAngle(double angleWrappedNow,
                          double& angleWrappedPrev,
                          double& angleUnwrapped) {
    double d = wrapToPi(angleWrappedNow - angleWrappedPrev);
    angleUnwrapped += d;
    angleWrappedPrev = angleWrappedNow;
}
```

### 10.3 Recommended foot yaw usage

| Use case | Use |
|---|---:|
| Constraint rotation using `sin/cos` | Either wrapped or unwrapped |
| Foot yaw error | `wrapToPi(desired - current)` or unwrapped branch |
| Foot yaw rate | Unwrapped or wrap-safe delta |
| Touchdown yaw prediction | Unwrapped |
| Swing yaw interpolation | Unwrapped or shortest-angle interpolation |

---

## 11. Swing Foot Touchdown Yaw Policy

During in-place turning, touchdown yaw should be continuous.

If command is yaw-rate only:

\[
\psi_{td} = \psi_u + \dot{\psi}_{cmd} T_{td} + \psi_{foot,offset}
\]

Code:

```cpp
double touchdownYawLeft_W =
    state.yaw_W_unwrapped
    + yawRateCmd * timeToTouchdown
    + leftFootYawOffset;

double touchdownYawRight_W =
    state.yaw_W_unwrapped
    + yawRateCmd * timeToTouchdown
    + rightFootYawOffset;
```

Do **not** wrap this if it will be used later for interpolation or comparison.

It is okay to pass it directly into `cos/sin`:

```cpp
double c = std::cos(touchdownYawLeft_W);
double s = std::sin(touchdownYawLeft_W);
```

### 11.1 Foot position offsets

For yaw-aware nominal foot placement:

\[
p_{foot,W}^{nom} = p_{body,W} + R_z(\psi_{td}) p_{foot,B}^{nom}
\]

Use:

```cpp
Eigen::Vector3d nominalFoot_W =
    bodyPosition_W + Rz(touchdownYaw_W) * nominalFootOffset_B;
```

This is safe with unwrapped yaw because `Rz()` is periodic.

---

## 12. Yaw Error Policy

Although the current command is yaw-rate only, some modules may still compute yaw error, for example:

- stabilizing torso yaw
- comparing desired foot yaw to current foot yaw
- debugging reference tracking
- future absolute heading mode

Never do raw subtraction between wrapped angles:

```cpp
// Bad
double yawError = yawDesiredWrapped - yawCurrentWrapped;
```

Use shortest-angle error:

```cpp
double yawError = wrapToPi(yawDesiredWrapped - yawCurrentWrapped);
```

Mathematically:

\[
e_{\psi} = \mathrm{wrapToPi}(\psi_d - \psi)
\]

If a continuous target yaw is needed near the current unwrapped yaw:

```cpp
double liftAngleNear(double targetWrapped, double currentUnwrapped) {
    return currentUnwrapped
         + wrapToPi(targetWrapped - wrapToPi(currentUnwrapped));
}
```

Mathematically:

\[
\psi_{target,u}
=
\psi_{current,u}
+
\mathrm{wrapToPi}
\left(
\psi_{target,w} - \mathrm{wrapToPi}(\psi_{current,u})
\right)
\]

---

## 13. What Should Be Changed in the Codebase

> [!NOTE]
> The exact filenames may differ slightly, but the implementation should follow this dependency flow:
>
> `StateEstimator` → `StateEstimate` → `DesiredFootPositions` / `ReferenceTrajectory` / `ConvexMPC` / `GaitScheduler`

---

### 13.1 Add yaw fields to `StateEstimate`

Add:

```cpp
double yaw_W_wrapped = 0.0;
double yaw_W_unwrapped = 0.0;
double yawRate_W = 0.0;
```

Optional:

```cpp
double leftFootYaw_W_wrapped = 0.0;
double rightFootYaw_W_wrapped = 0.0;
double leftFootYaw_W_unwrapped = 0.0;
double rightFootYaw_W_unwrapped = 0.0;
```

---

### 13.2 Add common angle utilities

Create or reuse a utility header:

```cpp
#pragma once

#include <cmath>

inline double wrapToPi(double x) {
    return std::atan2(std::sin(x), std::cos(x));
}

inline double unwrapFromPrevious(double wrappedNow,
                                 double wrappedPrev,
                                 double unwrappedPrev) {
    return unwrappedPrev + wrapToPi(wrappedNow - wrappedPrev);
}

inline double liftAngleNear(double targetWrapped,
                            double currentUnwrapped) {
    return currentUnwrapped
         + wrapToPi(targetWrapped - wrapToPi(currentUnwrapped));
}
```

---

### 13.3 Update yaw in the state estimator

Current risky logic may look like:

```cpp
state.yaw = extractYawFromQuaternion(q);
state.yawRate = (state.yaw - previousYaw) / dt;
```

Replace with:

```cpp
double yawWrappedNow = extractYawFromQuaternion(q);
double dyaw = wrapToPi(yawWrappedNow - state.yaw_W_wrapped);

state.yaw_W_unwrapped += dyaw;
state.yawRate_W = dyaw / dt;
state.yaw_W_wrapped = yawWrappedNow;
```

If the estimator has a trusted gyro-based yaw rate, you may use that for `yawRate_W`, but still maintain `yaw_W_unwrapped` using wrap-safe integration.

---

### 13.4 Use unwrapped yaw in MPC initial state

Replace:

```cpp
x0(yawIndex) = state.yaw_W_wrapped;
```

with:

```cpp
x0(yawIndex) = state.yaw_W_unwrapped;
```

If the old field is called `state.yaw`, decide whether to rename it or explicitly document that it is now unwrapped.

Recommended:

```cpp
state.yaw_W_unwrapped
```

is clearer than a generic:

```cpp
state.yaw
```

---

### 13.5 Use unwrapped yaw in MPC reference trajectory

For yaw-rate command:

```cpp
for (int k = 0; k < horizonLength; ++k) {
    double t = static_cast<double>(k + 1) * mpcDt;
    xRef[k](yawIndex) = state.yaw_W_unwrapped + yawRateCmd * t;
}
```

Remove any wrapping from the MPC yaw reference.

Search for patterns like:

```cpp
wrapToPi(yawRef)
std::fmod(yawRef, ...)
normalizeAngle(yawRef)
```

inside reference trajectory generation and remove them unless they are only used for logging.

---

### 13.6 Use unwrapped yaw in touchdown planning

Replace risky touchdown yaw logic like:

```cpp
touchdownYaw = wrapToPi(currentYaw + yawRateCmd * T_td);
```

with:

```cpp
touchdownYaw = state.yaw_W_unwrapped + yawRateCmd * T_td;
```

If nominal foot yaw offsets exist:

```cpp
leftTouchdownYaw_W =
    state.yaw_W_unwrapped + yawRateCmd * T_td + leftFootYawOffset;

rightTouchdownYaw_W =
    state.yaw_W_unwrapped + yawRateCmd * T_td + rightFootYawOffset;
```

---

### 13.7 Constraint builder policy

For `GaitScheduler.cpp` / constraint matrix construction:

```cpp
fillYawRotatedFootConstraintBlock(C_unit, footYaw_W, C_left, ...);
```

`footYaw_W` may be wrapped or unwrapped if the function only does:

```cpp
std::cos(footYaw_W);
std::sin(footYaw_W);
```

However, for consistency, pass unwrapped yaw from planning/state if available:

```cpp
fillYawRotatedFootConstraintBlock(C_unit, leftFootYaw_W_unwrapped, C_left, ...);
fillYawRotatedFootConstraintBlock(C_unit, rightFootYaw_W_unwrapped, C_right, ...);
```

This keeps the interface consistent and avoids accidental future misuse.

---

## 14. What Should Remain Wrapped

Keep wrapped yaw for:

### 14.1 Debug logs

```cpp
LOG("yaw_deg", rad2deg(wrapToPi(state.yaw_W_unwrapped)));
```

### 14.2 Viewer labels

```cpp
displayYaw = wrapToPi(state.yaw_W_unwrapped);
```

### 14.3 External absolute heading interfaces

If a future command gives absolute heading:

```cpp
yawTargetWrapped = command.heading;  // probably [-pi, pi]
```

then lift it before using it as a continuous reference:

```cpp
yawTargetUnwrapped = liftAngleNear(yawTargetWrapped,
                                   state.yaw_W_unwrapped);
```

### 14.4 Shortest angular error

```cpp
yawError = wrapToPi(yawDesired - yawCurrent);
```

---

## 15. Anti-Patterns to Remove

> [!CAUTION]
> Search the codebase for these patterns.

### 15.1 Raw yaw difference from wrapped yaw

Bad:

```cpp
double dyaw = yawNowWrapped - yawPrevWrapped;
```

Good:

```cpp
double dyaw = wrapToPi(yawNowWrapped - yawPrevWrapped);
```

---

### 15.2 Wrapping MPC yaw reference

Bad:

```cpp
psiRef[k] = wrapToPi(psiNow + yawRateCmd * t);
```

Good:

```cpp
psiRef[k] = psiNowUnwrapped + yawRateCmd * t;
```

---

### 15.3 Using wrapped yaw in MPC state

Bad:

```cpp
x0(yawIndex) = wrapToPi(currentYaw);
```

Good:

```cpp
x0(yawIndex) = currentYawUnwrapped;
```

---

### 15.4 Raw yaw error

Bad:

```cpp
yawError = desiredYaw - currentYaw;
```

Good for shortest-angle error:

```cpp
yawError = wrapToPi(desiredYaw - currentYaw);
```

Good for continuous MPC target:

```cpp
desiredYawUnwrapped = liftAngleNear(desiredYawWrapped,
                                    currentYawUnwrapped);
```

---

## 16. Testing and Verification Checklist

### 16.1 Log these values around 180 degrees

```cpp
yaw_W_wrapped
yaw_W_unwrapped
previous_yaw_W_wrapped
raw_dyaw = yaw_W_wrapped - previous_yaw_W_wrapped
safe_dyaw = wrapToPi(yaw_W_wrapped - previous_yaw_W_wrapped)
yawRate_W
x0_yaw_for_MPC
psiRef[0]
psiRef[1]
psiRef[last]
leftFootYaw_W
rightFootYaw_W
leftTouchdownYaw_W
rightTouchdownYaw_W
```

Expected around \(\pi\):

```text
yaw_W_wrapped:       3.13 -> -3.13
yaw_W_unwrapped:     3.13 ->  3.15
raw_dyaw:           -6.26
safe_dyaw:           0.02
yawRate_W:           normal value
psiRef[k]:           continuous, no +pi -> -pi jump
```

---

### 16.2 Test cases

#### Test A: Slow in-place yaw rotation through 180 degrees

Command:

```text
vx = 0
vy = 0
yawRateCmd = small constant
```

Expected:

- no fall near 180 degrees
- `yaw_W_unwrapped` continuous
- `yawRate_W` bounded
- MPC yaw reference continuous

#### Test B: Rotate beyond 360 degrees

Expected:

- `yaw_W_unwrapped` exceeds \(2\pi\)
- robot does not suddenly reverse direction
- logs display wrapped yaw normally

#### Test C: Alternating yaw-rate command

Command:

```text
yawRateCmd = +r for a few seconds
yawRateCmd = -r for a few seconds
```

Expected:

- yaw reference reverses smoothly
- no discontinuity at \(\pm\pi\)

#### Test D: Stance foot yaw near \(\pi\)

Expected constraint rows remain finite:

\[
c = \cos \pi = -1
\]

\[
s = \sin \pi = 0
\]

No singularity should appear in friction or CoP constraints.

---

## 17. Expected Root Cause of 180-Degree Falling

If the robot now rotates well up to about 180 degrees but falls near 180 degrees, the most likely cause is:

> [!WARNING]
> Some module is still using **wrapped yaw as a continuous value**.

Likely locations:

1. State estimator yaw-rate calculation
2. MPC initial yaw state
3. MPC yaw reference trajectory
4. Swing touchdown yaw prediction
5. Foot yaw interpolation
6. Body-frame/world-frame velocity transform using discontinuous yaw
7. Debug or safety logic accidentally feeding wrapped yaw back into control

The CoP/Friction constraint matrix itself should not become invalid at 180 degrees.

At \(\psi_f = \pi\):

\[
\cos\psi_f = -1
\]

\[
\sin\psi_f = 0
\]

The constraint rows simply flip direction. The feasible region remains physically valid.

---

## 18. Implementation Instructions for AI Agent

> [!IMPORTANT]
> Apply this policy consistently. Do not partially convert only one module.

### Task 1 — Add angle utility functions

Create or locate a common math utility header and add:

```cpp
inline double wrapToPi(double x);
inline double unwrapFromPrevious(double wrappedNow,
                                 double wrappedPrev,
                                 double unwrappedPrev);
inline double liftAngleNear(double targetWrapped,
                            double currentUnwrapped);
```

---

### Task 2 — Modify `StateEstimate`

Add explicit fields:

```cpp
double yaw_W_wrapped;
double yaw_W_unwrapped;
double yawRate_W;
```

Optionally add foot yaw fields if foot yaw is used for planning:

```cpp
double leftFootYaw_W_wrapped;
double rightFootYaw_W_wrapped;
double leftFootYaw_W_unwrapped;
double rightFootYaw_W_unwrapped;
```

---

### Task 3 — Update state estimator yaw logic

Replace raw yaw update with wrap-safe unwrapping.

Make sure:

```cpp
state.yaw_W_unwrapped
```

is continuous through \(\pm\pi\).

---

### Task 4 — Replace MPC yaw state input

All MPC state vectors should use:

```cpp
state.yaw_W_unwrapped
```

not wrapped yaw.

---

### Task 5 — Replace MPC yaw reference generation

For yaw-rate command:

```cpp
psiRef[k] = state.yaw_W_unwrapped + yawRateCmd * t;
```

Do not wrap `psiRef[k]`.

---

### Task 6 — Replace touchdown yaw prediction

Use:

```cpp
touchdownYaw_W = state.yaw_W_unwrapped + yawRateCmd * T_td + footYawOffset;
```

Do not wrap unless only printing.

---

### Task 7 — Keep wrapped yaw only for logs/UI/errors

Use:

```cpp
wrapToPi(state.yaw_W_unwrapped)
```

for logs and display.

Use:

```cpp
wrapToPi(a - b)
```

for shortest angular error.

---

### Task 8 — Audit the codebase

Search for:

```text
yaw
psi
heading
atan2
wrapToPi
normalizeAngle
fmod
```

For every occurrence, classify it as one of:

1. continuous control state → use unwrapped
2. display/interface → use wrapped
3. shortest-angle error → use wrapped difference
4. rotation matrix only → either is okay, prefer unwrapped source
5. constraint rotation only → either is okay, prefer unwrapped source

---

## 19. Final Policy

> [!IMPORTANT]
> The controller should have one continuous yaw timeline.

Use:

\[
\psi_u
\]

for all MPC/control/planning calculations.

Use:

\[
\psi_w = \mathrm{wrapToPi}(\psi_u)
\]

only when bounded angle representation is explicitly needed.

The safest implementation is:

```cpp
state.yaw_W_unwrapped;  // primary control yaw
state.yaw_W_wrapped;    // display/interface yaw
state.yawRate_W;        // wrap-safe yaw rate
```

For yaw-rate command:

\[
\psi_{ref}[k] = \psi_u[0] + \dot{\psi}_{cmd} t_k
\]

with no wrapping.

For friction/CoP constraints:

\[
c = \cos \psi_f, \qquad s = \sin \psi_f
\]

so wrapped and unwrapped yaw produce the same rotation, but using unwrapped yaw consistently reduces the chance of future branch mistakes.
