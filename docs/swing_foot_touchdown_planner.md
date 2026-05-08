# Swing Foot Touchdown Planner

This note explains how `SwingFootPlanner` computes the red touchdown marker used by swing feet. The planner is body-yaw aware, can operate in `fixed` or `realtime` target-update mode, and no longer uses the old `legacy_com_yaw_corrected` path.

Keyboard commands enter the controller through `user_command_filter` before they reach the planner. The same filter applies to walking `x_dot / y_dot / psi_dot` and standing `z_dot / roll / pitch`.

## 1. What the Planner Must Do

Touchdown targets should satisfy these constraints:

1. Forward motion should keep placing the next foot forward.
2. Backward motion should keep placing the next foot backward.
3. Lateral motion should shift the whole footprint left or right.
4. Once the robot stops, the average of both feet should converge to the desired body marker, `debug_body_target`.
5. When the robot is transitioning into a stop, touchdown should move slightly farther in the direction of travel so the robot brakes instead of coasting past the target.

The key design choice is:

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

```text
footCenter_W = (leftFoot_W + rightFoot_W) / 2
offset_B[leg] = Rz(yaw_des)^T * (footPos_W - footCenter_W)
offset_B[leg].x = 0
offset_B[leg].z = 0
```

This means the planner remembers only the left/right spacing between the feet. The forward/backward component is deliberately discarded so the feet can keep a clean stance line while moving forward or backward.

Example:

```text
left  offset_B = [0, +0.08, 0]
right offset_B = [0, -0.08, 0]
```

If `swing.nominal_foot_offsets_B` is present in YAML, that value is used directly and the runtime estimate is skipped.

## 4. Preview Time

The touchdown target is not computed from the current foot position alone. It is computed from where the body is expected to be when the next foot lands.

The preview horizon is:

```text
previewTime = (0.5 + bodyVelocityHalfStanceOffset) * stanceTime()
```

The intuition is simple:

- `0.5 * stanceTime` means "place the foot roughly halfway through the next stance interval."
- `bodyVelocityHalfStanceOffset` is a tuning knob.
  - `0` means plain half-stance preview.
  - Positive values look farther into the future and place the foot farther ahead.
  - Negative values look less far ahead and place the foot closer.

If `bodyVelocityHalfStanceOffset = 0.5`, the planner is looking one full stance interval ahead.

## 5. Swing Foot Yaw

The swing-foot yaw target is usually the torso yaw plus a yaw-lead term:

```text
yaw_target = yaw0 + swing_foot_yaw_lead_scale * psi_dot * previewTime
```

There is one extra heuristic for diagonal stepping. If the filtered lateral command is large enough, the planner uses the world direction from the current foot position to the touchdown target as the base yaw. That path is only enabled when:

- `|filtered_y_dot| >= 0.2 m/s`
- `filtered_x_dot != 0`
- the swing leg matches the lateral direction

If `filtered_x_dot < 0`, the yaw is rotated by `+90 deg` for the right foot and `-90 deg` for the left foot.

There is also a `psi_dot` bias:

- If `x_dot >= 0` and `psi_dot > 0`, the left foot gets a positive bias.
- If `x_dot >= 0` and `psi_dot < 0`, the right foot gets a negative bias.
- If `x_dot < 0`, the signs are flipped.

The bias magnitude is:

```text
min(20 deg, 100 * |psi_dot| deg)
```

This is intentionally a small correction. It adjusts the swing-foot heading, not the touchdown position.

## 6. Translation Yaw

Touchdown position uses a small yaw-averaging approximation:

```text
translationYaw = yaw0 + 0.5 * psi_dot * previewTime
step_W = Rz(translationYaw) * [x_dot, y_dot, 0] * previewTime
```

Why the `0.5`?

During the preview window, the body yaw changes from `yaw0` to `yaw0 + psi_dot * previewTime`. Using the midpoint yaw is a simple approximation to the full integral:

```text
step_W = integral from 0 to T of Rz(yaw0 + psi_dot * t) * v_cmd_B dt
```

For short preview intervals, the midpoint approximation is easier to reason about and is usually sufficient.

## 7. Final Touchdown Target

The core touchdown equation is:

```text
target_W = currentCenter_W
         + step_W
         + Rz(yaw0) * brakingOffset_B
         + Rz(yaw0) * nominalFootOffsets_B[leg]
```

Where:

- `currentCenter_W` is usually `_bodyTarget.position_W`.
- `step_W` is the previewed displacement from the command.
- `brakingOffset_B` is the optional stop/braking offset in body-yaw coordinates.
- `nominalFootOffsets_B[leg]` preserves the left/right stance spacing.

If the body target has not been seeded yet, the planner falls back to the last known footprint center.

## 8. Stop / Braking Offset

When the robot is slowing down, the planner can add an extra body-frame offset that helps the next touchdown brake the motion.

The logic is:

1. Watch the filtered planar command.
2. If command magnitude is dropping toward zero, capture a braking offset.
3. Reuse that offset for the next touchdown calculation.

Relevant YAML keys:

- `stop_capture_point_gain`
- `stop_capture_point_max_offset`
- `stop_velocity_deadband`

If you want to bypass the capture-point calculation and force a fixed value, set `swing.stop_braking_offset_B` in YAML. When that key is present, the planner uses the configured value instead of the capture-point estimate.

## 9. Why the Planner Reconstructs the Footprint Center

The planner does not use the previous foot target directly as the next foot's initial point.

Example:

```text
left touchdown  = [x=0.10, y=+0.08]
next right p_init = left touchdown
right touchdown = [x=0.20, y=+0.08]
```

That would collapse the stance geometry because the right foot would walk onto the left foot's lateral line.

Instead, the planner reconstructs the footprint center:

```text
center = touchdown - Rz(yaw_des) * nominalFootOffsets_B[leg]
```

Once the center is known, the next foot target is computed from that center plus the opposite foot offset. This keeps the left/right spacing stable across steps.

## 10. Behavior by User Command

### 10.1 Zero command / stopping

When `space` clears the raw keyboard command, the filtered command decays gradually. The body marker and MPC reference do not snap to zero instantly.

The planner may then add a braking offset. That offset is only latched when the filtered planar command is actually dropping or is already near zero. It is not recomputed every tick.

### 10.2 Forward motion: `x_dot > 0`, `y_dot = 0`, `psi_dot = 0`

```text
step_W = Rz(yaw0) * [x_dot, 0, 0] * previewTime
target_W = bodyTarget_W + step_W + Rz(yaw0) * nominalFootOffsets_B[leg]
```

Result:

- The future footprint center moves forward.
- Left and right feet stay on opposite sides of that center.

### 10.3 Backward motion: `x_dot < 0`, `y_dot = 0`, `psi_dot = 0`

```text
step_W = Rz(yaw0) * [x_dot, 0, 0] * previewTime
target_W = bodyTarget_W + step_W + Rz(yaw0) * nominalFootOffsets_B[leg]
```

Result:

- The future footprint center moves backward.
- The swing foot target moves behind the current support line.

### 10.4 Lateral motion: `y_dot != 0`

```text
step_W = Rz(yaw0) * [0, y_dot, 0] * previewTime
target_W = bodyTarget_W + step_W + Rz(yaw0) * nominalFootOffsets_B[leg]
```

Result:

- Positive `y_dot` moves the footprint center toward the body-left side.
- Negative `y_dot` moves it toward the body-right side.
- Nominal left/right spacing is preserved.

### 10.5 In-place turning: `x_dot = 0`, `y_dot = 0`, `psi_dot != 0`

```text
turnBias_W = Rz(yaw0) * [d_turn, 0, 0]
target_W = bodyTarget_W + turnBias_W + Rz(yaw0) * nominalFootOffsets_B[leg]
```

Result:

- The desired body marker moves slightly forward by the turn bias.
- The feet rotate around that biased marker.
- Swing-foot yaw still follows body yaw and yaw lead unless the diagonal-step heuristic activates.

### 10.6 Moving while turning: `x_dot / y_dot != 0`, `psi_dot != 0`

```text
step_W = Rz(yaw0 + 0.5 * psi_dot * previewTime) * [x_dot, y_dot, 0] * previewTime
target_W = bodyTarget_W + step_W + Rz(yaw0) * nominalFootOffsets_B[leg]
```

Result:

- The translation is approximated using the midpoint yaw over the preview interval.
- Left/right stance spacing is still defined in the body-yaw frame.
- The swing target therefore follows the future heading instead of the instantaneous one.
- When the raw command drops to zero, the braking offset turns on during the decay and turns off again once the command stabilizes.

## 11. What This Planner Deliberately Does Not Do

- It does not recompute the braking offset on every tick.
- It does not keep changing the touchdown target after a swing has already started.
- In `fixed` mode, the touchdown target is latched once for the swing and then held.
- In `realtime` mode, the target may be refreshed, but the current implementation still treats the touchdown geometry as a step-local decision.
- A zero keyboard command does not automatically trigger braking unless the filtered command actually decreases.

## 12. Summary

- Touchdown targets are world-frame positions.
- Nominal left/right spacing lives in the body-yaw frame.
- Previewing uses the expected future body position, not the current foot position alone.
- Swing-foot yaw is body-yaw aware, with a diagonal-step heuristic and a signed `psi_dot` bias.
- Stopping adds a braking offset so the feet help bring the body marker back over the support footprint.

This is the smallest frame convention that keeps the planner readable and consistent with the rest of the controller.
