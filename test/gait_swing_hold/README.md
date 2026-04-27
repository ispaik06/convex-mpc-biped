# Gait Swing Hold Test

## What This Test Is

`gait_swing_hold` is a manual MIT humanoid probe for isolating gait-scheduled swing-foot control on both legs.

It starts directly from the `copied_state` keyframe in `models/mit_humanoid/scene.xml`. The runner copies only the keyframe `qpos`, applies the optional torso `z` offset, then freezes every MuJoCo coordinate except the leg joints. The normal `RobotRunner` joint-position initializer is not used.

## What It Exercises

- Robot: MIT humanoid only (`m`).
- Model: loaded from `gait_swing_hold_test.xml_path` in `config/mit_humanoid/my_controller.yaml`, falling back to `model.xml_path` if unset.
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
Load MIT model
  -> read copied_state qpos from scene.xml
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

Headless:

```sh
./build/test/gait_swing_hold/main_gait_swing_hold_test m n
```

With torso height offset:

```sh
./build/test/gait_swing_hold/main_gait_swing_hold_test m n 0.2
```

Arguments:

- `m`: MIT humanoid. This test rejects other robot IDs.
- `y`: GUI viewer.
- `n`: headless.
- Optional third argument: torso base `z` offset in meters, applied immediately after loading `copied_state`.

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

This test explicitly activates the MIT humanoid config before constructing the controller:

```text
config/mit_humanoid/my_controller.yaml
```

Important sections:

- `gait_swing_hold_test.xml_path`: MuJoCo scene XML loaded by `GaitSwingHoldTestRunner`.
- `model.xml_path`: fallback MuJoCo scene XML if the test-specific path is empty.
- `timing`: swing/stance durations.
- `swing`: swing height, gains, touchdown target mode defaults.
- `foot_placement`: placement feedback/clamps.
- `gait_swing_hold_test.touchdown_target_mode`: overrides the touchdown target mode for this test.

## Notes

- This target is intentionally not added to CTest.
- This is a manual inspection/debug target, not a pass/fail regression test.
- The fixed coordinates are frozen by MuJoCo state projection, not by controller torques.
