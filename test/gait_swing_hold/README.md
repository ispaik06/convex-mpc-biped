# Gait Swing Hold Test

## What This Test Is

`gait_swing_hold` is a manual probe for isolating gait-scheduled swing-foot control on MIT humanoid, Unitree G1, and Unitree H1.

It starts directly from the scene keyframe configured in `gait_swing_hold_test.keyframe_name`. The runner copies only that keyframe `qpos`, applies the optional torso `z` offset, then freezes every MuJoCo coordinate except the leg joints. The normal `RobotRunner` joint-position initializer is not used.

## What It Exercises

- Robot: MIT humanoid (`m`), Unitree G1 (`g`), or Unitree H1 (`h`).
- Model: loaded from `gait_swing_hold_test.xml_path` in the active robot's `my_controller.yaml`, falling back to `model.xml_path` if unset.
- Runner: `GaitSwingHoldTestRunner`.
- Controller: `GaitSwingHoldController`.
- Frozen state: floating base, arms, and any other non-leg coordinates.
- Legs: both legs alternate between Cartesian swing-foot tracking and joint-PD hold according to `GaitScheduler`.

This is useful when debugging:

- Swing foot desired vs actual tracking. The CSV trace currently records the left leg as the debug channel.
- Touchdown target visualization through the left-leg `debug_left_touchdown_target` marker in `scene_test.xml`.
- Swing touchdown target formulas.
- Cartesian swing gains and leg operational-space dynamics.
- Whether foot sites and end-effector Jacobians are wired correctly.

## Execution Flow

```text
Load configured model
  -> read configured scene keyframe qpos
  -> apply optional torso z offset
  -> cache frozen qpos for all non-leg coordinates
  -> run GaitSwingHoldController directly, without RobotRunner initialization
  -> clamp frozen qpos every MuJoCo step
  -> write swing trace samples to CSV
```

The runner also zeros frozen DOF velocities during clamping so MuJoCo does not accumulate velocity on coordinates that are being position-projected.

## Build

From the repository root:

```sh
cmake --build build --target main_gait_swing_hold_test
```

## Run

GUI:

```sh
./build/test/gait_swing_hold/main_gait_swing_hold_test m y
```

G1:

```sh
./build/test/gait_swing_hold/main_gait_swing_hold_test g y
```

H1:

```sh
./build/test/gait_swing_hold/main_gait_swing_hold_test h y
```

Headless:

```sh
./build/test/gait_swing_hold/main_gait_swing_hold_test m n
```

With torso height offset:

```sh
./build/test/gait_swing_hold/main_gait_swing_hold_test m n 0.2
```

Arguments:

- `m`: MIT humanoid.
- `g`: Unitree G1.
- `h`: Unitree H1.
- `y`: GUI viewer.
- `n`: headless.
- Optional third argument: torso base `z` offset in meters, applied immediately after loading the configured keyframe.

Headless mode runs until interrupted.

## Outputs

The controller writes:

```text
build/gait_swing_hold_trace.csv
```

Columns:

```text
segment,time,phase,desired_x,desired_y,desired_z,actual_x,actual_y,actual_z
```

The file is overwritten each run.

## Relevant Config

This test explicitly activates the selected robot config before constructing the controller:

```text
config/mit_humanoid/my_controller.yaml
config/unitree_robots/g1/my_controller.yaml
config/unitree_robots/h1/my_controller.yaml
```

Important sections:

- `gait_swing_hold_test.xml_path`: MuJoCo scene XML loaded by `GaitSwingHoldTestRunner`.
- `gait_swing_hold_test.keyframe_name`: scene keyframe used to seed the frozen initial state.
- `model.xml_path`: fallback MuJoCo scene XML if the test-specific path is empty.
- `timing`: swing/stance durations.
- `swing`: swing height and gains.

## Notes

- This target is intentionally not added to CTest.
- This is a manual inspection/debug target, not a pass/fail regression test.
- The fixed coordinates are frozen by MuJoCo state projection, not by controller torques.
