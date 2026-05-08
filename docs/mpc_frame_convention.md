# MPC Frame Convention and Cost Weighting

This note explains how the reduced-body MPC separates world-frame dynamics from body-yaw interpretation of the cost. The short version is:

1. Keep the dynamics and targets in world coordinates.
2. Interpret command semantics, nominal offsets, and planar tracking costs in the body-yaw frame.
3. Keep the cost transform fixed from the reference yaw so the QP stays convex.

## 1. Current Implementation Summary

The current reduced-body MPC state is ordered as:

```text
x = [
  roll, pitch, yaw,
  px, py, pz,
  omega_x, omega_y, omega_z,
  vx, vy, vz,
  g
]
```

The current code interprets these pieces as follows:

- `x0[0:3]`: torso orientation, where yaw is the world heading.
- `x0[3:6]`: reduced-body COM position in world coordinates.
- `x0[6:9]`: torso angular velocity in world coordinates.
- `x0[9:12]`: reduced-body COM velocity in world coordinates.
- `x0[12]`: gravity scalar.

The main pipeline is:

- `ReferenceTrajectory` builds `Rz(psi_ref) * u_cmd_B` to obtain a world-frame velocity reference.
- `MPCFormulation` builds the dynamics matrix in world coordinates.
- `ConvexMPC::buildQP()` assembles `stateError = A_qp * x0 - X_ref` directly in world coordinates and applies a fixed diagonal state weight.

That last point is the one that matters most.

If the configuration uses anisotropic weights such as `px != py` or `vx != vy`, those values are effectively world-axis weights. When the robot yaws, the meaning of "forward" and "lateral" changes with the body, but the weight matrix stays aligned to world x/y.

## 2. Why Fixed World Weights Can Become Problematic

Consider a diagonal cost:

```text
J = (x - x_ref)^T Q_world (x - x_ref)
```

If `Q_world` is diagonal and anisotropic, the cost is tied to world axes.

That leads to the following issue during turning:

- "Track body-lateral position" becomes "track world-y position."
- "Track body-forward velocity" becomes "track world-x velocity."
- Foot placement already rotates with yaw, but the torso tracking objective does not.
- The support geometry and the body-tracking objective can drift apart as yaw changes.

In practice, this is exactly the kind of mismatch that can make a robot feel stable when walking straight and loose or underdamped when turning.

## 3. Recommended Frame Split

### Keep these in world coordinates

- COM position
- COM velocity
- world gravity
- contact forces and moments
- the final touchdown target position
- the world-frame dynamics matrices

Why:

- Foot contact and COM position are geometric world quantities.
- The current dynamics formulation is already assembled in world coordinates.
- World-frame debug markers are easier to interpret.
- Moving the entire MPC into a body frame would require re-deriving contact offsets, inertia, and wrench constraints together.

### Interpret these in the body-yaw frame

- user commands
- nominal foot offsets
- capture-point braking offset
- planar position cost meaning
- planar velocity cost meaning
- horizontal angular-velocity cost meaning
- tilt cost meaning

The body-yaw frame is only a bookkeeping frame. It is not a full six-degree-of-freedom body frame, and it should not be treated as one. Keeping only yaw avoids mixing roll/pitch tilt with the vertical axis in the reduced-body controller.

## 4. Recommended MPC Cost Formulation

Keep the dynamics and the targets in world coordinates, but transform the cost error with the reference yaw at each horizon step.

Current form:

```text
J = Σ (x_k - x_ref_k)^T Q_world (x_k - x_ref_k)
```

Recommended form:

```text
J = Σ e_k^T Q_world e_k
e_k = T_k (x_k - x_ref_k)
```

Where `T_k` is a fixed step-local transform built from the reference yaw `psi_ref_k`.

Because `T_k` is fixed from the reference trajectory, the problem remains a convex quadratic program.

### Useful block transforms

For planar position, planar velocity, and horizontal angular velocity, use a yaw rotation:

```text
R_k = Rz(psi_ref_k)^T

T_position   = diag(R_k, 1)
T_velocity   = diag(R_k, 1)
T_omega      = diag(R_k, 1)
T_orientation = I_3
```

In matrix form, the 3x3 planar rotation block is:

```text
[ cos ψ   sin ψ   0 ]
[ -sin ψ  cos ψ   0 ]
[   0       0     1 ]
```

The safest first implementation is:

- rotate position, velocity, and angular velocity blocks
- leave roll/pitch/yaw as raw Euler errors

That matches the current Euler-angle convention and avoids mixing orientation conventions.

### Orientation block

The state stores Euler `[roll, pitch, yaw]`, so roll and pitch should remain raw attitude errors for now. Yaw is a heading scalar and can remain wrapped as needed. If a later revision wants a more exact attitude cost, it should use a proper orientation-error representation rather than trying to rotate raw Euler angles with a planar transform.

## 5. Implementation Point

The smallest code change is to modify `ConvexMPC::buildQP()` so that the weighted state assembly uses the transformed error blocks instead of a world-fixed diagonal penalty.

In pseudocode:

```text
for each horizon step k:
    psi_ref = reference yaw at step k
    T_k = block transform built from psi_ref
    e_k = T_k * (x_k - x_ref_k)
    accumulate Q_k = T_k^T * Q_world * T_k
```

This preserves convexity while making the cost semantics follow the robot heading.

## 6. Contact Wrench Constraints

Turning instability is not always a cost-frame problem.

`GaitScheduler::buildConstraintMatrices()` currently applies friction and wrench limits directly to the wrench decision variables:

```text
[Fx, Fy, Fz, Mx, My, Mz]
```

That is fine as long as the constraint geometry matches the foot frame. In particular, these quantities should be checked in a foot-local or foot-yaw frame:

- foot half length
- foot half width
- roll-moment limits
- pitch-moment limits

If `Mx` and `My` are treated as fixed world axes while the foot yaw changes, the foot length and width axes no longer line up with the constraint interpretation.

Recommended direction:

- keep force and moment variables in world coordinates
- rotate the constraint evaluation into the foot contact frame when using foot geometry

If the robot still behaves oddly after the cost frame fix, this is the next place to inspect.

## 7. Quick Validation Sequence

Before changing the cost transform, first check whether isotropic weights improve turning stability:

1. Set `px == py`.
2. Set `vx == vy`.
3. If possible, also set `roll == pitch` and `omega_x == omega_y`.
4. Compare straight walking, turning in place, and curved walking.

If the robot becomes noticeably more stable, the issue is likely that the cost is locked to world axes.

Then apply the step-local yaw transform and restore the anisotropic weights.

If turning still looks strange after that, inspect the contact wrench constraints in the foot frame.

## 8. Summary

- Keep desired body target and final foot target in world coordinates.
- Keep user command semantics and nominal offsets in the body-yaw frame.
- Keep dynamics in world coordinates.
- Apply the MPC cost transform only to position, velocity, and angular-velocity blocks.
- Leave roll/pitch/yaw as raw Euler attitude errors for now.
- Revisit the wrench constraints if turning still behaves poorly after the cost fix.

This is the smallest change that preserves the existing controller structure while giving the cost the right physical meaning during yaw motion.
