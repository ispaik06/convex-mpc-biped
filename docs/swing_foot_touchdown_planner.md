# Swing Foot Touchdown Planner

This note explains how `SwingFootPlanner` computes the red touchdown marker used by swing feet. The planner is body-yaw aware, latches a touchdown target once per swing, and no longer uses the old `legacy_com_yaw_corrected` path.

Keyboard commands enter the controller through `user_command_filter` before they reach the planner. The same filtered command stream updates the body target, and the planner consumes the resulting body position/yaw when placing touchdown targets.

> [!IMPORTANT]
> The planner keeps **touchdown position** in world coordinates, **nominal spacing** in the body-yaw frame, and **swing-foot yaw** as a separate heading problem.
>
> The nominal foot offset is rotated using the estimated yaw at touchdown.

## 1. What the Planner Must Do

Touchdown targets should satisfy these constraints:

1. Forward motion should keep placing the next foot forward.
2. Backward motion should keep placing the next foot backward.
3. Lateral motion should shift the whole footprint left or right.
4. Once the robot stops, the average of both feet should converge to the desired body marker, `debug_body_target`.
5. When the robot is transitioning into a stop, touchdown should move slightly farther in the direction of travel so the robot brakes instead of coasting past the target.

The key design choice is:

> [!NOTE]
> The planner deliberately moves the **touchdown target** toward the desired body marker instead of pulling the body marker toward the feet.

```text
Do not pull the desired body marker toward the feet.
Instead, move the swing-foot touchdown target toward the desired body marker.
```

That matters because the reduced-body MPC reference is built from `_bodyTarget`. If the desired body marker is already ahead of the feet and the feet remain behind, the MPC keeps moving the COM toward the marker while the support polygon lags behind. That is an easy way to lose balance during stopping.

## 2. Coordinate Frames

### World frame `W`

- Simulation/world coordinates.
- The final touchdown target is always returned as a world position `target_W`.

### Body-yaw frame `B`

- A bookkeeping frame that ignores roll and pitch and keeps only yaw aligned to the robot heading.
- User commands `[x_dot, y_dot]` are interpreted in this frame.
- Nominal foot offsets are stored in this frame.

The current code uses the desired body yaw from the SRB reference when possible, not a full floating-base pose. In practice, touchdown position and swing-foot yaw are computed around the desired body marker that the MPC is trying to track.

## 3. Nominal Foot Offsets

When the planner is reset, the first call to `desiredFootPositions()` captures the current left and right foot positions and stores their body-yaw-frame offsets as `_nominalFootOffsets_B[leg]`.

The procedure is:

$$
p_{\text{center}}^W = \frac{p_{\text{left}}^W + p_{\text{right}}^W}{2}
$$

$$
o_{\text{leg}}^B = R_z(\psi_{\text{des}})^\top \left(p_{\text{foot}}^W - p_{\text{center}}^W\right)
$$

$$
\left(o_{\text{leg}}^B\right)_x = 0,\qquad \left(o_{\text{leg}}^B\right)_z = 0
$$

This means the planner remembers only the left/right spacing between the feet. The forward/backward component is deliberately discarded so the feet can keep a clean stance line while moving forward or backward.

Example:

$$
o_{\text{left}}^B = \begin{bmatrix} 0 \\ +0.08 \\ 0 \end{bmatrix},\qquad
o_{\text{right}}^B = \begin{bmatrix} 0 \\ -0.08 \\ 0 \end{bmatrix}
$$

If `swing.nominal_foot_offsets_B` is present in YAML, that value is used directly and the runtime estimate is skipped.

## 4. Preview Time

The touchdown target is not computed from the current foot position alone. It is computed from where the body is expected to be when the next foot lands.

The preview horizon is:

$$
T_{\text{preview}} = \left(0.5 + \alpha_{\text{half-stance}}\right) T_{\text{stance}}
$$

Here, $\alpha_{\text{half-stance}}$ maps to `bodyVelocityHalfStanceOffset`, and
$T_{\text{stance}}$ maps to `stanceTime()`.

The intuition is simple:

- `0.5 * stanceTime` means "place the foot roughly halfway through the next stance interval."
- `bodyVelocityHalfStanceOffset` is a tuning knob.
  - `0` means plain half-stance preview.
  - Positive values look farther into the future and place the foot farther ahead.
  - Negative values look less far ahead and place the foot closer.

If `bodyVelocityHalfStanceOffset = 0.5`, the planner is looking one full stance interval ahead.

## 5. Swing Foot Yaw

The swing-foot yaw target is usually the torso yaw plus a yaw-lead term:

$$
\psi_{\text{foot}}^{\text{des}} = \psi_0 + k_{\text{lead}} \dot{\psi} \, T_{\text{preview}}
$$

Here, `k_{\text{lead}}` maps to `swing_foot_yaw_lead_scale`.

There is one extra heuristic for diagonal stepping. If the filtered lateral command is large enough, the planner uses the world direction from the current foot position to the touchdown target as the base yaw. That path is only enabled when:

- `|filtered_y_dot| >= 0.2 m/s`
- `filtered_x_dot != 0`
- the swing leg matches the lateral direction

If `filtered_x_dot < 0`, the yaw is rotated by `+90 deg` for the right foot and `-90 deg` for the left foot.

There is also a `psi_dot` bias:

- If `x_dot >= 0` and `psi_dot > 0`, the left foot gets a positive bias.
- If `x_dot >= 0` and `psi_dot < 0`, the right foot gets a negative bias.
- If `x_dot < 0`, the bias sign is **not** flipped; only the diagonal-heading heuristic changes.

The bias magnitude is:

$$
|\Delta \psi| = \min\left(20^\circ,\ 100^\circ |\dot{\psi}|\right)
$$

This is intentionally a small correction. It adjusts the swing-foot heading, not the touchdown position.

## 6. Translation Yaw

Touchdown position uses a small yaw-averaging approximation:

$$
\psi_{\text{trans}} = \psi_0 + \frac{1}{2}\dot{\psi} T_{\text{preview}}
$$

$$
\Delta p_W = R_z(\psi_{\text{trans}})\begin{bmatrix}\dot{x}\\\dot{y}\\0\end{bmatrix} T_{\text{preview}}
$$

Why the `0.5`?

During the preview window, the body yaw changes from `yaw0` to `yaw0 + psi_dot * previewTime`. Using the midpoint yaw is a simple approximation to the full integral:

$$
\Delta p_W = \int_0^{T_{\text{preview}}} R_z(\psi_0 + \dot{\psi} t)
\begin{bmatrix}\dot{x}\\\dot{y}\\0\end{bmatrix}\, dt
$$

For short preview intervals, the midpoint approximation is easier to reason about and is usually sufficient.

## 7. Final Touchdown Target

The core touchdown equation is:

$$
\begin{aligned}
p_{\text{td}}^W &= p_{\text{center}}^W + \Delta p_W \\
                &\quad + R_z(\psi_0) p_{\text{brake}}^B \\
                &\quad + R_z(\psi_0 + \dot{\psi} T_{\text{td}}) \left(p_{\text{nom}}^B[\text{leg}] + p_{\text{turn}}^B[\text{leg}]\right)
\end{aligned}
$$

Where:

- $p_{\text{center}}^W$ corresponds to `currentCenter_W` in the code.
- $\Delta p_W$ corresponds to `step_W`.
- $p_{\text{brake}}^B$ corresponds to `brakingOffset_B`.
- $p_{\text{nom}}^B[\text{leg}]$ corresponds to `nominalFootOffsets_B[leg]`.
- $p_{\text{turn}}^B[\text{leg}]$ corresponds to the turning lead term.
- $T_{\text{td}}$ corresponds to `remainingSwingTime`.

This nominal offset term is evaluated once when the touchdown target is latched for the swing.

If the body target has not been seeded yet, the planner falls back to the last known footprint center.

### 7.1 Tangential Lead for Turning

When `turn_tangential_lead_scale` is nonzero, the planner adds a yaw-rate-dependent lead in the
body-frame `x` direction:

$$
p_{\text{turn}}^B[\text{leg}] =
\begin{bmatrix}
-k_{\text{turn}} \, \dot{\psi} \, (p_{\text{nom}}^B[\text{leg}])_y \, T_{\text{preview}} \\
0 \\
0
\end{bmatrix}
$$

The full touchdown target then uses the combined body-frame offset:

$$
p_{\text{td}}^W =
p_{\text{center}}^W + \Delta p_W + R_z(\psi_0) p_{\text{brake}}^B +
R_z(\psi_0 + \dot{\psi} T_{\text{td}}) \left(p_{\text{nom}}^B[\text{leg}] + p_{\text{turn}}^B[\text{leg}]\right)
$$

This is the "step into the turn" heuristic:

- `turn_tangential_lead_scale > 0` makes the swing foot lead forward or backward in proportion to yaw rate and lateral foot offset.
- Positive `psi_dot` pushes the right foot forward and the left foot backward.
- Setting the gain to `0` disables the extra lead.

## 8. Stop / Braking Offset

When the robot is slowing down, the planner can add an extra body-frame offset that helps the next touchdown brake the motion.

Let the planar braking offset be

$$
p_{\text{brake}}^B = \begin{bmatrix} b_x \\ b_y \end{bmatrix}.
$$

It enters the touchdown target as

$$
p_{\text{td}}^W \leftarrow p_{\text{td}}^W + R_z(\psi_0)\begin{bmatrix}b_x\\b_y\\0\end{bmatrix}.
$$

Only the planar `x/y` components are used by the current planner. If `swing.stop_braking_offset_B` is set in YAML, that value is used directly and the capture-point estimate is skipped.

> [!TIP]
> If the robot should brake harder on the next step, the relevant knobs are `stop_capture_point_gain`, `stop_capture_point_max_offset`, `stop_velocity_deadband`, `stop_braking_latch_clear_ticks`, and an optional fixed `swing.stop_braking_offset_B`.

### 8.1 When the braking offset is active

The planner watches the filtered planar command

$$
c_B = \begin{bmatrix}\dot{x}\\\dot{y}\end{bmatrix},
\qquad
c_{B,\text{prev}} = \begin{bmatrix}\dot{x}_{\text{prev}}\\\dot{y}_{\text{prev}}\end{bmatrix}.
$$

Let $\epsilon_c$ denote the deadband threshold used by the planner.

The braking offset is considered active when the previous command was already outside the deadband and the current command is either near zero or is shrinking:

$$
a_{\text{brake}} =
\begin{cases}
0, & \|c_{B,\text{prev}}\| \le \epsilon_c, \\
1, & \|c_B\| \le \epsilon_c, \\
1, & \|c_B\| + 10^{-9} < \|c_{B,\text{prev}}\|, \\
0, & \text{otherwise.}
\end{cases}
$$

This means the offset is latched when the command starts decaying into a stop. The first valid braking offset for that stop episode is cached, then reused until the command has stayed inside the deadband long enough to clear the latch.

### 8.2 Capture-point estimate

If no fixed YAML override is present, the planner estimates a braking offset from the reduced-body COM velocity.

The symbols below map to YAML keys:

- $k_{\text{cp}}$ maps to `stop_capture_point_gain`
- $r_{\max}$ maps to `stop_capture_point_max_offset`
- $\epsilon_c$ maps to `stop_velocity_deadband`
- $n_{\text{clear}}$ maps to `stop_braking_latch_clear_ticks`

First, it computes the world-frame reduced-body COM velocity:

$$
v_{\text{com}}^W = v_{\text{torso}}^W + \omega^W \times \left(R_z(\psi_0) r_{\text{com}}^B\right)
$$

where `reducedBodyComVelocityWorld()` combines the torso linear velocity and the angular contribution from the body COM offset.

Then it projects that velocity into the body-yaw frame:

$$
v_{\text{com}}^B = R_z(\psi_0)^\top v_{\text{com}}^W.
$$

Let
$$
h_{\text{com}} = \max\!\left(10^{-6},\ p_{\text{com},z}^W\right), \qquad
\omega_0 = \sqrt{\frac{|g|}{h_{\text{com}}}}.
$$

The raw capture-point-style offset is

$$
p_{\text{brake,raw}}^B =
\frac{k_{\text{cp}}}{\omega_0}
\begin{bmatrix}
v_{\text{com},x}^B \\
v_{\text{com},y}^B
\end{bmatrix}.
$$

This is the usual linear inverted pendulum capture-point scaling: larger COM velocity gives a larger braking step, and the gain lets you make it more or less aggressive.

The planner then clamps the norm:

$$
p_{\text{brake}}^B =
\begin{cases}
p_{\text{brake,raw}}^B, & \|p_{\text{brake,raw}}^B\| \le r_{\max}, \\
\dfrac{r_{\max}}{\|p_{\text{brake,raw}}^B\|} p_{\text{brake,raw}}^B, & \text{otherwise}.
\end{cases}
$$

### 8.3 Deadband behavior

`stop_velocity_deadband` is used in two places:

1. To decide whether the filtered planar command is close enough to zero for braking to be active.
2. To ignore tiny computed braking offsets when the estimated COM motion is already negligible.

The first case controls when the latch may be refreshed or released. The second case suppresses tiny offsets that would otherwise jitter around zero.

If

$$
\left\|\begin{bmatrix}v_{\text{com},x}^B \\ v_{\text{com},y}^B\end{bmatrix}\right\|
\le \epsilon_c,
$$

the braking estimate is treated as negligible.

The deadband-hold counter is

$$
h_{k+1} =
\begin{cases}
0, & a_{\text{brake}} = 1, \\
h_k + 1, & a_{\text{brake}} = 0 \text{ and } \|c_B\| \le \epsilon_c, \\
0, & \|c_B\| > \epsilon_c.
\end{cases}
$$

The latched braking offset is cleared when

$$
h_k \ge n_{\text{clear}}.
$$

In words: once a braking offset has been latched, it stays live while the command is near zero, but it is cleared after `stop_braking_latch_clear_ticks` consecutive control ticks inside the deadband. If the command leaves the deadband before that, the hold counter resets. If the command rises back above the deadband, the latch is cleared immediately.

When the estimate is recomputed directly from COM velocity, the planner still returns zero if the reduced-body planar COM velocity is already negligible:

$$
\left\|\begin{bmatrix}v_{\text{com},x}^B \\ v_{\text{com},y}^B\end{bmatrix}\right\|
\le \epsilon_c
\quad\Rightarrow\quad
p_{\text{brake}}^B = 0.
$$

## 9. Why the Planner Reconstructs the Footprint Center

The planner does not use the previous foot target directly as the next foot's initial point.

Example:

$$
\begin{aligned}
p_{\text{td},L}^W &= \begin{bmatrix} 0.10 \\ +0.08 \end{bmatrix} \\
p_{\text{init},R}^W &= p_{\text{td},L}^W \\
p_{\text{td},R}^W &= \begin{bmatrix} 0.20 \\ +0.08 \end{bmatrix}
\end{aligned}
$$

That would collapse the stance geometry because the right foot would walk onto the left foot's lateral line.

Instead, the planner reconstructs the footprint center:

$$
p_{\text{center}}^W = p_{\text{td}}^W - R_z(\psi_{\text{des}}) p_{\text{nom}}^B[\text{leg}]
$$

Once the center is known, the next foot target is computed from that center plus the opposite foot offset. This keeps the left/right spacing stable across steps.

## 10. Behavior by User Command

### 10.1 Zero command / stopping

When `space` clears the raw keyboard command, the filtered command decays gradually. The body marker and MPC reference do not snap to zero instantly.

The planner may then add a braking offset. That offset is only latched when the filtered planar command is actually dropping or is already near zero. It stays latched until the filtered command has remained inside the deadband for `stop_braking_latch_clear_ticks` consecutive ticks, or until the command leaves the stop region and the latch is cleared immediately.

### 10.2 Forward motion: `x_dot > 0`, `y_dot = 0`, `psi_dot = 0`

$$
\Delta p_W = R_z(\psi_0)\begin{bmatrix}\dot{x}\\0\\0\end{bmatrix} T_{\text{preview}}
$$

$$
p_{\text{td}}^W = p_{\text{body}}^W + \Delta p_W + R_z(\psi_0 + \dot{\psi} T_{\text{td}}) p_{\text{nom}}^B[\text{leg}]
$$

Result:

- The future footprint center moves forward.
- Left and right feet stay on opposite sides of that center.

### 10.3 Backward motion: `x_dot < 0`, `y_dot = 0`, `psi_dot = 0`

$$
\Delta p_W = R_z(\psi_0)\begin{bmatrix}\dot{x}\\0\\0\end{bmatrix} T_{\text{preview}}
$$

$$
p_{\text{td}}^W = p_{\text{body}}^W + \Delta p_W + R_z(\psi_0 + \dot{\psi} T_{\text{td}}) p_{\text{nom}}^B[\text{leg}]
$$

Result:

- The future footprint center moves backward.
- The swing foot target moves behind the current support line.

### 10.4 Lateral motion: `y_dot != 0`

$$
\Delta p_W = R_z(\psi_0)\begin{bmatrix}0\\\dot{y}\\0\end{bmatrix} T_{\text{preview}}
$$

$$
p_{\text{td}}^W = p_{\text{body}}^W + \Delta p_W + R_z(\psi_0 + \dot{\psi} T_{\text{td}}) p_{\text{nom}}^B[\text{leg}]
$$

Result:

- Positive `y_dot` moves the footprint center toward the body-left side.
- Negative `y_dot` moves it toward the body-right side.
- Nominal left/right spacing is preserved.

### 10.5 In-place turning: `x_dot = 0`, `y_dot = 0`, `psi_dot != 0`

$$
\mathbf{v}_{\text{cmd}}^B = \begin{bmatrix}0\\0\\0\end{bmatrix}
$$

$$
\Delta p_W = \mathbf{0}
$$

$$
p_{\text{td}}^W =
p_{\text{center}}^W + R_z(\psi_0) p_{\text{brake}}^B +
R_z(\psi_0 + \dot{\psi} T_{\text{td}}) \left(p_{\text{nom}}^B[\text{leg}] + p_{\text{turn}}^B[\text{leg}]\right)
$$

Result:

- Pure turning now adds a tangential lead in the touchdown position.
- The right foot steps slightly forward and the left foot slightly backward when `psi_dot > 0`.
- The tangential lead is controlled by `turn_tangential_lead_scale`.
- The nominal foot offset uses the estimated yaw at touchdown, where `T_td = remainingSwingTime`.
- Turning-specific heading adjustment happens later in
  `My_Controller::swingFootYawFromDiagonalStepHeading()`, where `psiBias_W` is added to the
  swing-foot yaw.

### 10.6 Moving while turning: `x_dot / y_dot != 0`, `psi_dot != 0`

$$
\Delta p_W = R_z\!\left(\psi_0 + \frac{1}{2}\dot{\psi} T_{\text{preview}}\right)
\begin{bmatrix}\dot{x}\\\dot{y}\\0\end{bmatrix} T_{\text{preview}}
$$

$$
p_{\text{td}}^W =
p_{\text{body}}^W + \Delta p_W +
R_z(\psi_0 + \dot{\psi} T_{\text{td}}) \left(p_{\text{nom}}^B[\text{leg}] + p_{\text{turn}}^B[\text{leg}]\right)
$$

Result:

- The translation is approximated using the midpoint yaw over the preview interval.
- Left/right stance spacing is still defined in the body-yaw frame.
- The nominal foot offset is rotated using the heading expected at touchdown.
- The tangential lead is applied whenever `psi_dot != 0`.
- When the raw command drops to zero, the braking offset turns on during the decay and then stays latched for a short deadband hold window before turning off again.

## 11. What This Planner Deliberately Does Not Do

- It does not recompute the braking offset on every tick.
- It does not keep a braking offset latched forever once the command has sat inside the deadband long enough.
- It does not keep changing the touchdown target after a swing has already started.
- It does not expose a `fixed` / `realtime` touchdown-target mode switch anymore.
- A zero keyboard command does not automatically trigger braking unless the filtered command actually decreases.

## 12. Summary

- Touchdown targets are world-frame positions.
- Nominal left/right spacing lives in the body-yaw frame.
- Previewing uses the expected future body position, not the current foot position alone.
- Swing-foot yaw is body-yaw aware, with a diagonal-step heuristic and a signed `psi_dot` bias.
- Turning can optionally add a tangential lead to the touchdown position via `turn_tangential_lead_scale`.
- Stopping adds a braking offset so the feet help bring the body marker back over the support footprint.

## 13. Code References

- Nominal foot offsets and the reconstructed stance center are implemented in [SwingFootPlanner::ensureNominalFootOffsets](../My_Controller/src/SwingFootPlanner.cpp#L91), [SwingFootPlanner::currentFootTouchdownTarget](../My_Controller/src/SwingFootPlanner.cpp#L136), and [SwingFootPlanner::recordSequentialTouchdown](../My_Controller/src/SwingFootPlanner.cpp#L282).
- Preview-time logic lives in [SwingFootPlanner::touchdownPreviewTime](../My_Controller/src/SwingFootPlanner.cpp#L149), and the full touchdown target equation is assembled in [SwingFootPlanner::touchdownTargetWorldBodyVelocityHalfStance](../My_Controller/src/SwingFootPlanner.cpp#L245).
- Swing-foot yaw and the diagonal-step heuristic are implemented in [MyController::swingFootYawTargetWorld](../My_Controller/src/My_Controller.cpp#L917), [swingFootYawFromDiagonalStepHeading](../My_Controller/src/My_Controller.cpp#L47), and [MyController::swingFootYawPsiOffset](../My_Controller/src/My_Controller.cpp#L29).
- The tangential turn lead and the final touchdown offset are implemented in [SwingFootPlanner::touchdownOffsetBodyFrame](../My_Controller/src/SwingFootPlanner.cpp#L154), [SwingFootPlanner::touchdownTargetWorldBodyVelocityHalfStance](../My_Controller/src/SwingFootPlanner.cpp#L245), and [SwingFootPlanner::recordSequentialTouchdown](../My_Controller/src/SwingFootPlanner.cpp#L282). The turning gain comes from [ControllerConfig.h](../My_Controller/include/MyController/ControllerConfig.h#L62) and [ControllerConfig.cpp](../My_Controller/src/ControllerConfig.cpp#L253).
- The braking-offset capture-point estimate, deadband gating, and latch-clear logic are implemented in [SwingFootPlanner::shouldApplyBrakingOffset](../My_Controller/src/SwingFootPlanner.cpp#L173), [SwingFootPlanner::computeStopBrakingOffsetBodyFrame](../My_Controller/src/SwingFootPlanner.cpp#L192), [SwingFootPlanner::stopBrakingOffsetBodyFrame](../My_Controller/src/SwingFootPlanner.cpp#L237), and the latch block inside [SwingFootPlanner::desiredFootPositions](../My_Controller/src/SwingFootPlanner.cpp#L291). The tuning knobs come from [ControllerConfig.h](../My_Controller/include/MyController/ControllerConfig.h#L66) and [ControllerConfig.cpp](../My_Controller/src/ControllerConfig.cpp#L265).

This is the smallest frame convention that keeps the planner readable and consistent with the rest of the controller.
