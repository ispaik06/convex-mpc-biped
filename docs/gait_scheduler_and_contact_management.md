# Gait Scheduler and Contact Management

This note explains how the controller turns a periodic gait definition into:

1. a binary stance/swing schedule,
2. horizon contact constraints for the MPC QP, and
3. a reactive contact overlay that can freeze touchdown targets, search for late contact, and soften force bounds during transitions.

> [!IMPORTANT]
> **`GaitScheduler` is the nominal schedule generator.**  
> **`ContactManager` is the reactive layer that can override that schedule when measured contact disagrees with the nominal plan.**

## 1. What Each Part Owns

| Component | Main input | Main output |
| --- | --- | --- |
| `GaitScheduler` | `cycle`, `swing`, `stance`, horizon time, locomotion mode | Phase `p(s,t)`, stance flag `c(s,t)`, and horizon contact matrices `D`, `C`, `C_bound` |
| `ContactManager` | scheduled contact, measured contact, nominal foot targets | `activeContact`, `earlyContact`, `lateContact`, `searchModeActive`, `contactRampAlpha`, and managed foot targets |
| `My_Controller` | filtered user command, state estimate, gait/contact outputs | Reference trajectory, horizon override, MPC solve input, and stance-wrench blending |

> [!NOTE]
> The nominal gait is periodic, but the reactive layer is allowed to disagree with it for a few ticks if that keeps the robot safer.

## 2. Time Base and Periodic Phase

The scheduler is anchored by `HorizonClock`. Its job is to keep a cycle-relative origin `t0` so the gait phase is evaluated consistently as time advances.

Let

$$
T_c = \texttt{cycleTime()}, \qquad
T_{\text{sw}} = \texttt{swingTime()}, \qquad
T_{\text{st}} = \texttt{stanceTime()},
$$

with the configuration check enforcing

$$
T_c = T_{\text{sw}} + T_{\text{st}}.
$$

The MPC horizon uses

$$
t_k = t_0 + k\,\Delta t, \qquad \Delta t = \texttt{dtMpc()},
$$

where `t0` is the synchronized cycle origin and `k` is the horizon step index.

For each side `s`, the nominal phase is written with the fractional-part operator:

$$
\varphi_s(t) = \operatorname{frac}\!\left(\frac{t - t_0}{T_c} + \phi_s\right),
$$

where

$$
\operatorname{frac}(x) = x - \lfloor x \rfloor.
$$

with the side offset

$$
\phi_{\text{Left}} = \frac{1}{2}, \qquad \phi_{\text{Right}} = 0.
$$

This means the left and right legs are half a cycle apart.

> [!TIP]
> With the checked-in default `stance / cycle = 0.6`, the gait has a **double-support overlap** of
>
> $$
> T_{\text{ds}} = \max(0,\ 2T_{\text{st}} - T_c)
> $$
>
> which is `0.2 T_c` for the default 0.6 stance fraction.

## 3. Nominal Stance / Swing Decision

The stance indicator is

$$
c_s(t) =
\begin{cases}
1, & 0 \le \varphi_s(t) < \dfrac{T_{\text{st}}}{T_c}, \\
0, & \text{otherwise.}
\end{cases}
$$

In standing mode, the scheduler short-circuits the logic:

$$
\varphi_s(t) = 0, \qquad c_s(t) = 1
\quad \text{for both sides.}
$$

So `bothFeetStance(t)` is simply

$$
c_{\text{both}}(t) = c_{\text{Left}}(t) \wedge c_{\text{Right}}(t).
$$

> [!NOTE]
> The phase is **nominal**, not measured. It is the schedule the controller expects to be true unless contact sensing says otherwise.

## 4. Horizon Contact Matrices

For each horizon step `k`, `GaitScheduler::buildConstraintMatrices()` evaluates contact at `t_k` and fills three objects:

$$
D \in \mathbb{R}^{12H \times 12H}, \qquad
C \in \mathbb{R}^{24H \times 12H}, \qquad
C_{\text{bound}} \in \mathbb{R}^{24H},
\qquad
H = \texttt{horizonSteps()}.
$$

The wrench decision variable at one step is ordered as

$$
w_k =
\begin{bmatrix}
F_{L,x} & F_{L,y} & F_{L,z} &
F_{R,x} & F_{R,y} & F_{R,z} &
M_{L,x} & M_{L,y} & M_{L,z} &
M_{R,x} & M_{R,y} & M_{R,z}
\end{bmatrix}^\top.
$$

The controller then enforces:

$$
C w_k \le C_{\text{bound}}, \qquad
D w_k = 0,
$$

stacked across the full horizon.

### 4.1 What `C` Means

`C` contains the stance-foot inequality constraints:

$$
\lvert F_x \rvert \le \mu F_z, \qquad
\lvert F_y \rvert \le \mu F_z
$$

$$
F_{z,\min} \le F_z \le F_{z,\max}
$$

$$
\lvert M_x \rvert \le \ell_w F_z, \qquad
\lvert M_y \rvert \le \ell_\ell F_z
$$

$$
\lvert M_z \rvert \le \tau_{\text{tor}} \, \mu F_z
$$

where:

- $\mu$ maps to `mpc.frictionCoefficient`
- $\ell_w$ maps to `mpc.footHalfWidth`
- $\ell_\ell$ maps to `mpc.footHalfLength`
- $\tau_{\text{tor}}$ maps to `mpc.torsionalFrictionScale`

The bounds vector is the linearized version of those same rules. The lower normal-force bound is what gets softened near contact transitions.

### 4.2 What `D` Means

`D` is the equality side of the horizon contact problem.

- If a leg is in **swing**, its force and moment variables are driven to zero through equality rows.
- If a leg is in **stance**, `C` carries the wrench limits and `D` contributes no zero-wrench rows for that leg.

In other words, `D` is the "swing-leg zero wrench" selector.

> [!WARNING]
> If the contact topology changes, the nonzero pattern of `D` changes as well.  
> `ConvexMPC` treats that as a contact-signature change and cold-starts OSQP.

### 4.3 Foot-Local Moment Handling

When the active wrench model is `NoRollMoment`, the controller injects each foot's local x-axis into `GaitScheduler` before the constraint matrices are built.

That means the scheduler can express the stance-foot roll-moment direction in the current foot frame instead of assuming a fixed world-axis interpretation.

This is why the main controller does:

1. read the current foot frame axes,
2. give them to the scheduler,
3. rebuild the constraints,
4. then solve the QP.

## 5. How the Reactive Contact Manager Sits on Top

The contact manager does not replace the nominal gait. It watches the nominal schedule and the measured contact state, then applies corrections when those disagree.

### 5.1 State Vocabulary

| State | Meaning |
| --- | --- |
| `scheduledContact` | What the gait phase says should be happening |
| `estimatedContact` | What the measured contact signal says is happening |
| `activeContact` | Final contact flag used by the controller |
| `earlyContact` | Measured contact appeared before the nominal touchdown |
| `lateContact` | Nominal touchdown happened but measured contact is still missing |
| `searchModeActive` | The planner is searching downward for ground after a late touchdown |
| `contactRampAlpha` | Ramp factor used to soften wrench feedforward and minimum normal force |
| `frozenTouchdownPosition_W` | Touchdown anchor used when contact must be preserved or searched |
| `commandedFootTarget_W` | The actual target returned to the leg controller |

> [!IMPORTANT]
> The contact manager changes **both** the near-term MPC constraints and the swing-foot reference.
> It is not just a logging aid.

### 5.2 Estimated Contact

If a force signal is available, contact is estimated from the measured normal force with hysteresis:

$$
\hat c_{k+1} =
\begin{cases}
1, & n_k \ge F_{\text{on}} \text{ for } N_{\text{on}} \text{ confirm ticks}, \\
0, & n_k \le F_{\text{off}} \text{ for } N_{\text{off}} \text{ confirm ticks}, \\
\hat c_k, & \text{otherwise.}
\end{cases}
$$

where `F_on`, `F_off`, `N_on`, and `N_off` come from the contact-manager parameters.

If force sensing is not available, the code falls back to the raw foot contact bit.

### 5.3 Early Contact

Early contact means:

$$
\text{scheduledContact} = 0,\qquad
\text{estimatedContact} = 1,
$$

and the leg has actually been released during swing.

The intent is simple:

- the foot touched down early,
- the planner should stop trying to place it somewhere else,
- the controller should freeze the touchdown target at the real contact point.

So the commanded foot target becomes:

$$
p_{\text{cmd}}^W = p_{\text{freeze}}^W = p_{\text{foot}}^W.
$$

### 5.4 Late Contact

Late contact means:

$$
\text{scheduledContact} = 1,\qquad
\text{estimatedContact} = 0.
$$

This is the opposite failure mode: the scheduler expects stance, but the foot has not actually found support yet.

When late contact is detected, the manager:

1. freezes the touchdown anchor,
2. enters search mode,
3. lowers the commanded target in `z` by a small amount every tick,
4. stops when contact is re-established or the search limit is reached.

The search depth evolves as

$$
d_{\text{search}}(t+\Delta t) =
\min\!\left(d_{\max},\ d_{\text{search}}(t) + v_{\text{search}} \Delta t\right),
$$

so the search target is

$$
p_{\text{search}}^W =
\begin{bmatrix}
x_{\text{freeze}} \\
y_{\text{freeze}} \\
z_{\text{freeze}} - d_{\text{search}}
\end{bmatrix}.
$$

The `lateContactTime` counter is used to flag recovery failure if the search lasts too long.

### 5.5 Contact Ramp

When a contact becomes active, the controller does not immediately jump to full stance wrench.

Instead, it uses

$$
\alpha_{\text{ramp}} =
\operatorname{clamp}\!\left(\frac{t_{\text{ramp}}}{T_{\text{ramp}}},\,0,\,1\right)
$$

with `T_ramp = contactRampDuration`.

That same factor is used in two places:

1. **stance-wrench feedforward** in the low-level controller,
2. **minimum normal-force scaling** inside the horizon override.

So near touchdown:

$$
F_{z,\min}^{\text{eff}} = \alpha_{\text{ramp}} \, F_{z,\min}.
$$

When the contact is inactive, the scale is zero.

## 6. How ContactManager Modifies the Controller Inputs

The controller consumes the contact manager in two ways.

### 6.1 Managed Foot Positions

`managedFootPositions()` returns the nominal desired feet unless a reactive state needs to override them.

Piecewise, the idea is:

$$
p_{\text{des,managed}}^W =
\begin{cases}
p_{\text{search}}^W, & \text{late contact / search mode}, \\
p_{\text{freeze}}^W, & \text{early contact}, \\
p_{\text{nom}}^W, & \text{otherwise.}
\end{cases}
$$

This is what keeps the swing-foot planner from chasing a target that is no longer physically meaningful.

### 6.2 Horizon Override

`buildHorizonOverride()` creates a short look-ahead override of length `contactLockSteps`.

For each of those initial horizon steps, it writes:

$$
\tilde c_{\text{Left}} = c_{\text{active,Left}}, \qquad
\tilde c_{\text{Right}} = c_{\text{active,Right}}
$$

and

$$
F_{z,\min}^{\text{eff}} =
\begin{cases}
\alpha_{\text{ramp}} F_{z,\min}, & \text{if the leg is active}, \\
0, & \text{otherwise.}
\end{cases}
$$

So the MPC sees a near-term contact plan that matches the current measured state instead of blindly trusting the nominal phase.

> [!NOTE]
> This override is local to the first `contactLockSteps` horizon samples.
> It does **not** rewrite the whole periodic gait.

## 7. How the Main Controller Uses Both

The main controller follows this sequence during an MPC update:

1. Ask `ContactManager` for the managed foot positions.
2. Ask `ContactManager` for the horizon override.
3. Give the override to `GaitScheduler::buildConstraintMatrices()`.
4. Build the reference trajectory from the managed foot positions.
5. Build the reduced-body MPC dynamics and cost.
6. Solve the QP.

So the full flow is:

$$
\text{Nominal gait} \rightarrow \text{Reactive contact overlay} \rightarrow \text{Reference and constraints} \rightarrow \text{MPC solve}
$$

This separation is deliberate:

- `GaitScheduler` stays purely periodic.
- `ContactManager` stays reactive.
- The MPC sees the result of both.

## 8. What to Look At When Debugging

- If the robot is walking but the schedule looks wrong, inspect `p_s(t)` and `c_s(t)`.
- If the robot touches down early, check `earlyContact` and whether the managed foot target froze.
- If the robot misses the ground, check `lateContact`, `searchModeActive`, and `lateContactTime`.
- If the QP suddenly cold-starts, the nonzero pattern of `D` probably changed.
- If stance feels too aggressive around touchdown, check `contactRampAlpha` and the effective `F_{z,\min}` scaling.

## 9. Summary

- `GaitScheduler` gives a clean periodic stance/swing pattern.
- `ContactManager` handles the real-world mismatch between the nominal schedule and measured contact.
- `C` enforces stance-foot wrench limits.
- `D` zeroes swing-leg wrenches and changes the QP contact signature when topology changes.
- The near-term contact override and the managed foot target both keep the MPC and the swing planner aligned with what the robot is actually doing.
