# Reference Trajectory

This note explains how the controller builds the horizon-wise body reference that drives the SRB MPC.

The key idea is simple:

1. seed the trajectory from the current desired body pose,
2. advance planar position in the body-yaw frame,
3. advance yaw only when the gait is not in double support,
4. carry the resulting body pose into the horizon state and foot-relative geometry.

> [!IMPORTANT]
> **`ReferenceTrajectory` is not a standalone planner.**
> It unrolls a horizon from the current body target and filtered user command.
>
> The yaw rule is delegated to `BodyMotionReference`, which uses the gait schedule as a gate.

## 1. What the Trajectory Contains

`ReferenceTrajectoryOutput` stores four horizon-length objects:

| Field | Shape | Meaning |
| --- | --- | --- |
| `X_ref` | `13H` | Reference reduced-body state stacked by horizon step |
| `r_left` | `3 x H` | Left foot position relative to the reference body pose |
| `r_right` | `3 x H` | Right foot position relative to the reference body pose |
| `psi` | `H` | Reference yaw samples |
| `tk` | `H` | Horizon timestamps |

where `H = horizonSteps()`.

The reduced-body state order is

$$
x =
\begin{bmatrix}
\phi & \theta & \psi &
p_x & p_y & p_z &
\omega_x & \omega_y & \omega_z &
v_x & v_y & v_z &
g
\end{bmatrix}^\top.
$$

> [!NOTE]
> The position and velocity blocks are world-frame quantities.
> The command semantics that generate them are interpreted in the body-yaw frame.

## 2. Seed State and Command Inputs

The controller does not build the horizon from the measured torso pose alone.
Before `ReferenceTrajectory::build()` is called, the controller splices the current desired body pose into the seed state:

- orientation comes from the body target yaw,
- position comes from the body target position,
- the rest of the seed remains aligned with the current reduced-body state.

The command input is

$$
u_{\text{cmd}}^B =
\begin{bmatrix}
\dot x & \dot y & \dot z
\end{bmatrix}^\top,
\qquad
\dot \psi_{\text{cmd}} \in \mathbb{R}.
$$

The controller forwards the filtered `z_dot` command into the trajectory builder.
That keeps the seed body height and the horizon vertical velocity consistent with the
current keyboard input.

## 3. Yaw Reference

Yaw is not integrated blindly at every step.
The helper in `BodyMotionReference` asks the gait schedule whether the robot is in double support at the current sample time.

Define the sample time used for yaw gating as

$$
t_{s,k} =
\begin{cases}
t_k, & k = 0, \\
t_k - \dfrac{1}{2}\Delta t, & k \ge 1.
\end{cases}
$$

The reason for the half-step lookback is practical:

- the schedule is periodic but piecewise,
- exact phase boundaries are not where you want the gate to chatter,
- midpoint sampling makes the yaw gate less sensitive to boundary alignment.

Let

$$
c_{\text{both}}(t) = c_{\text{Left}}(t) \wedge c_{\text{Right}}(t)
$$

be the nominal double-support predicate from the gait scheduler.

Then the yaw gate is

$$
g_k =
\begin{cases}
0, & c_{\text{both}}(t_{s,k}) = 1, \\
1, & c_{\text{both}}(t_{s,k}) = 0.
\end{cases}
$$

So the yaw recurrence is

$$
\psi_k = \mathrm{wrap}\!\left(\psi_{k-1} + g_k \, \dot\psi_{\text{cmd}} \, \Delta t\right),
$$

with

$$
\mathrm{wrap}(\alpha) = \mathrm{atan2}(\sin\alpha, \cos\alpha).
$$

The seed values are

$$
\psi_0 = \psi_{\text{seed}}, \qquad
p_0^W = p_{\text{seed}}^W.
$$

In words:

- during double support, yaw is frozen,
- during single-support swing intervals, yaw is advanced by the commanded rate.

> [!TIP]
> This yaw gate is why the implementation belongs in `BodyMotionReference`, not in the gait scheduler note.
> The scheduler owns the phase predicate; the trajectory helper owns the integration rule.

## 4. Planar Position Propagation

The reference body position is propagated in world coordinates by rotating the body-frame planar command through the current reference yaw:

$$
p_{k+1}^W =
p_k^W +
R_z(\psi_k)
\begin{bmatrix}
\dot x_{\text{cmd}} \\
\dot y_{\text{cmd}} \\
0
\end{bmatrix}
\Delta t.
$$

So planar translation is always integrated, but the direction of travel follows the current reference heading.

The vertical position is not integrated in this helper.
The seed height is preserved in `p_z`, which keeps the reference consistent with the reduced-body model used by the MPC.

The corresponding world velocity is

$$
v_k^W =
R_z(\psi_k)
\begin{bmatrix}
\dot x_{\text{cmd}} \\
\dot y_{\text{cmd}} \\
\dot z_{\text{cmd}}
\end{bmatrix}.
$$

That velocity is written directly into the state reference block.

## 5. Horizon State Assembly

For each horizon sample `k`, the builder fills the reference state as:

$$
x_k^{\text{ref}} =
\begin{bmatrix}
\phi_k \\
\theta_k \\
\psi_k \\
p_{x,k} \\
p_{y,k} \\
p_{z,k} \\
0 \\
0 \\
\dot\psi_k \\
v_{x,k} \\
v_{y,k} \\
v_{z,k} \\
g
\end{bmatrix}.
$$

The implementation keeps:

- roll and pitch from the seed,
- yaw from the gated yaw integrator,
- angular velocity roll/pitch at zero,
- angular velocity yaw at the gated yaw rate,
- linear velocity from the body-command rotation,
- gravity copied from the seed.

This is intentionally conservative.
The reference is not trying to model all possible torso motions, only the reduced-body motion needed by the MPC.

## 6. Foot-Relative Geometry

The foot offsets are converted into horizon-relative vectors:

$$
r_{\text{left},k}^W = p_{\text{left,des}}^W - p_k^W,
\qquad
r_{\text{right},k}^W = p_{\text{right,des}}^W - p_k^W.
$$

These are not swing-foot trajectories.
They are the contact lever arms used by the SRB dynamics.

In the MPC, those offsets appear through cross-product terms such as

$$
r \times F,
$$

which is why the reference body pose and the touchdown planner need to remain consistent.

> [!NOTE]
> If the body reference shifts, the moment arms shift too.
> That changes the reduced-body dynamics seen by the QP.

## 7. Why This Helper Exists Separately

`ReferenceTrajectory` is a horizon assembler.
`BodyMotionReference` is the small motion kernel that knows how to:

| Helper | Role |
| --- | --- |
| `shouldAdvanceYaw()` | Check whether the gait is in double support |
| `advanceYaw()` | Apply gated yaw integration with wrap-around |
| `yawRate()` | Emit the gated yaw-rate reference block |
| `advancePlanarPosition()` | Integrate planar body motion in world coordinates |
| `worldVelocity()` | Rotate the body-frame command into a world-frame velocity |

This split keeps the controller structure readable:

- the gait scheduler decides the nominal stance/swing timing,
- the body-motion helper turns that timing into a yaw gate,
- the reference trajectory packages the result into the MPC inputs.

## 8. Practical Reading of the Code

If you are debugging a trajectory plot:

- `psi[k]` tells you which yaw the MPC is being asked to track at horizon step `k`,
- `tk[k]` tells you when that sample lives on the horizon,
- `r_left[k]` and `r_right[k]` tell you where the feet are relative to the body marker,
- `X_ref` tells you the full reduced-body target state the QP sees.

If yaw looks too aggressive, the first thing to check is whether the robot is spending too much time outside double support or whether `psi_dot` is being filtered too weakly upstream.

## 9. Code References

- Yaw gating and the recurrence for `psi_k` are implemented in [BodyMotionReference::shouldAdvanceYaw](../My_Controller/src/BodyMotionReference.cpp#L16), [BodyMotionReference::advanceYaw](../My_Controller/src/BodyMotionReference.cpp#L20), and [BodyMotionReference::yawRate](../My_Controller/src/BodyMotionReference.cpp#L31); they are called from [ReferenceTrajectory::build](../My_Controller/src/ReferenceTrajectory.cpp#L8).
- Planar propagation of `p_k^W` and `v_k^W` lives in [BodyMotionReference::advancePlanarPosition](../My_Controller/src/BodyMotionReference.cpp#L35) and is consumed in [ReferenceTrajectory::build](../My_Controller/src/ReferenceTrajectory.cpp#L8).
- Horizon assembly for `X_ref`, `psi`, `tk`, `r_left`, and `r_right` is done in [ReferenceTrajectory::build](../My_Controller/src/ReferenceTrajectory.cpp#L8).
- The body-target seed that the trajectory starts from is updated in [MyController::updateBodyTarget](../My_Controller/src/My_Controller.cpp#L695).

## 10. Summary

- `ReferenceTrajectory` turns a body target and a filtered command into horizon arrays.
- Yaw advances only when the gait is not in double support.
- Planar position is integrated in the body-yaw frame and written to world coordinates.
- The foot offsets are horizon-relative contact lever arms, not swing-foot path points.
- `BodyMotionReference` exists to keep the yaw gate and planar propagation reusable and easy to inspect.
