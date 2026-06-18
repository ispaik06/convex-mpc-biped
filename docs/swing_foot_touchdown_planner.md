# Swing Foot Touchdown Planner

This note documents the current `SwingFootPlanner` touchdown target equation.

The planner is now reduced to a basic Raibert-style preview:

- zero planar command should step at the nominal foot offsets,
- pure in-place turning should rotate the nominal offsets,
- translational walking should use only filtered command preview,
- the touchdown timing is `0.5 + offset`, with optional mid/high-speed switches.

## 1. Inputs and Latching

`MyController::runController()` updates the filtered user command, updates the body target, passes
that body target into `SwingFootPlanner`, and then asks the planner for desired foot positions.

During walking, each swing leg gets a touchdown target only when it has just left stance or when no
valid target exists yet. After that, the target is cached for the rest of that swing:

```cpp
const bool shouldUpdateTouchdownTarget = wasInStance || !_touchdownTargetValid[leg];
```

For stance legs, the planner returns the cached touchdown target. If no cache exists yet, it seeds
from the current measured foot position.

## 2. Frames

- `W`: world frame. The final touchdown target is returned as `target_W`.
- `B`: body-yaw frame. This is a planar frame using the controller's body yaw target, not full
  roll/pitch.

The planner stores nominal left/right foot spacing in `B`, while final foot positions are stored in
`W`.

## 3. Nominal Foot Offsets

On the first valid call, the planner records the current stance width unless
`swing.nominal_foot_offsets_B` is provided in YAML.

Runtime inference:

$$
p_{\text{center}}^W = \frac{p_L^W + p_R^W}{2}
$$

$$
o_{\text{leg}}^B =
R_z(\psi_0)^T \left(p_{\text{foot,leg}}^W - p_{\text{center}}^W\right)
$$

Then the forward and vertical components are discarded:

$$
o_{\text{leg},x}^B = 0,\qquad o_{\text{leg},z}^B = 0
$$

So the nominal offset is mainly the lateral stance width:

$$
o_L^B = [0,\ +w,\ 0]^T,\qquad o_R^B = [0,\ -w,\ 0]^T
$$

## 4. Preview Time

The preview time is:

$$
T_p = \max\left(0,\ \left(0.5 + k_{\text{half}}\right) T_{\text{stance}}\right)
$$

where `k_half` is selected as:

- `swing.body_velocity_half_stance_offset` when filtered planar speed is at or below
  `swing.body_velocity_half_stance_offset_switch_speed`
- `swing.mid_speed_body_velocity_half_stance_offset` when filtered planar speed is above
  `swing.body_velocity_half_stance_offset_switch_speed`
  and at or below `swing.high_speed_body_velocity_half_stance_offset_switch_speed`
- `swing.high_speed_body_velocity_half_stance_offset` when filtered planar speed is above
  `swing.high_speed_body_velocity_half_stance_offset_switch_speed`

## 5. Current Touchdown Equation

The filtered planar command is:

$$
v_{\text{cmd}}^B =
\begin{bmatrix}
\dot{x}_{cmd} \\
\dot{y}_{cmd} \\
0
\end{bmatrix}
$$

Translation preview uses this command:

$$
\psi_{\text{trans}} = \psi_0 + \frac{1}{2}\dot{\psi}_{cmd}T_p
$$

$$
\Delta p_{\text{step}}^W =
R_z(\psi_{\text{trans}}) v_{\text{cmd}}^B T_p
$$

The touchdown yaw used for nominal foot spacing is based on remaining swing time:

$$
T_{\text{td}} =
\operatorname{clamp}\left(T_{\text{cycle}}\left(1 - p_{\text{gait}}\right),\ 0,\ T_{\text{swing}}\right)
$$

$$
\psi_{\text{td}} = \psi_0 + \dot{\psi}_{cmd} T_{\text{td}}
$$

The body-frame planned offset is:

$$
o_{\text{planned}}^B =
R_z(\psi_0)^T \Delta p_{\text{step}}^W
+ R_z(\psi_{\text{td}} - \psi_0) o_{\text{leg}}^B
$$

The final target is:

$$
p_{\text{td}}^W = p_{\text{center}}^W + R_z(\psi_0)o_{\text{planned}}^B
$$

and `z` is forced to `-0.005`.

## 6. Lateral Crossing Guard

The planner only prevents a foot from crossing to the opposite side of the centerline:

```cpp
if (nominalLateralOffset > 0.0) {
    plannedOffset_B.y() = std::max(plannedOffset_B.y(), 0.0);
} else if (nominalLateralOffset < 0.0) {
    plannedOffset_B.y() = std::min(plannedOffset_B.y(), 0.0);
}
```

It does not force the rotated nominal offset back to the original lateral width. That matters for
pure yaw stepping, where `Rz(psi_td - psi0) * o_leg_B` should remain the nominal rotated offset.

## 7. Behavior by Command

### Zero planar command

When `x_dot = 0` and `y_dot = 0`:

$$
\Delta p_{\text{step}}^W = 0
$$

So the target is the current center plus nominal foot spacing. This is the expected in-place stepping
behavior after pressing `space`.

The controller also snaps filtered `x_dot`, `y_dot`, and `psi_dot` to zero when the raw command is
fully zero, so `space` no longer leaves a decaying filtered command that can pull touchdown targets
backward.

### Pure in-place turning

When `x_dot = 0`, `y_dot = 0`, and `psi_dot != 0`:

$$
\Delta p_{\text{step}}^W = 0
$$

The target is only:

$$
p_{\text{td}}^W =
p_{\text{center}}^W +
R_z(\psi_0)R_z(\psi_{\text{td}} - \psi_0)o_{\text{leg}}^B
$$

So pure turning rotates the nominal left/right offsets around the footprint center, without the old
extra tangential lead.

### Translating

When `x_dot` or `y_dot` is nonzero, translation preview comes from the filtered command:

$$
\Delta p_{\text{step}}^W =
R_z(\psi_{\text{trans}})
[\dot{x}_{cmd},\ \dot{y}_{cmd},\ 0]^T T_p
$$

## 8. Code References

- Command filtering and zero-command snap: [My_Controller/src/My_Controller.cpp](../My_Controller/src/My_Controller.cpp#L598)
- Walking body target anchor: [My_Controller/src/My_Controller.cpp](../My_Controller/src/My_Controller.cpp#L711)
- Planner call site: [My_Controller/src/My_Controller.cpp](../My_Controller/src/My_Controller.cpp#L1370)
- `space` raw command reset: [common/src/Utilities/KeyboardCommand.cpp](../common/src/Utilities/KeyboardCommand.cpp#L355)
- Full touchdown equation: [My_Controller/src/SwingFootPlanner.cpp](../My_Controller/src/SwingFootPlanner.cpp#L243)
- Swing target latching: [My_Controller/src/SwingFootPlanner.cpp](../My_Controller/src/SwingFootPlanner.cpp#L322)
