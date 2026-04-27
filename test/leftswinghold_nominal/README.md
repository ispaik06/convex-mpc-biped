# Left Swing Hold Nominal Pose Probe

## What This Test Is

`leftswinghold_nominal` is a non-viewer probe that records foot positions from the `copied_state` keyframe.

It does not run `LeftSwingHoldController` and does not perform swing control. Its purpose is to inspect the keyframe geometry used when tuning left-swing touchdown targets.

## What It Exercises

- Robot: MIT humanoid only (`m`).
- Model: loaded from `left_swing_hold_test.xml_path` in `config/mit_humanoid/my_controller.yaml`, falling back to `model.xml_path` if unset.
- Pose source: `copied_state` keyframe qpos in `models/mit_humanoid/scene.xml`.
- Output: one CSV row per leg.

This is useful when debugging:

- The copied-state foot site positions.
- Foot collision-geom center positions.
- Foot coordinates in the yaw-aligned reduced-body frame `B`.
- Differences between copied-state foot positions and the legacy nominal foot-placement seed.

## Execution Flow

```text
Load MIT model
  -> setup RobotParams and MuJoCo bindings
  -> copy copied_state qpos into mjData
  -> update reduced-body mass/COM properties
  -> read foot site and foot collision-geom center positions
  -> transform positions into yaw-aligned body frame B
  -> print values and write CSV
```

## Build

From the repository root:

```sh
cmake --build build --target main_left_swing_hold_nominal_test
```

## Run

Default output path:

```sh
./build/test/leftswinghold_nominal/main_left_swing_hold_nominal_test m
```

Custom output path:

```sh
./build/test/leftswinghold_nominal/main_left_swing_hold_nominal_test m /tmp/left_swing_nominal.csv
```

Arguments:

- `m`: MIT humanoid. This probe rejects other robot IDs.
- `csv-path`: optional output CSV path.

There is no GUI/headless flag because this probe does not launch the viewer.

## Outputs

Default CSV:

```text
build/left_swing_hold_nominal_pose.csv
```

The program also prints the same values to stdout.

CSV columns:

```text
leg,
site_Wx,site_Wy,site_Wz,
collision_geom_center_Wx,collision_geom_center_Wy,collision_geom_center_Wz,
site_Bx,site_By,site_Bz,
collision_geom_center_Bx,collision_geom_center_By,collision_geom_center_Bz,
legacy_nominal_Bx,legacy_nominal_By,legacy_nominal_Bz,
delta_site_legacy_x,delta_site_legacy_y,delta_site_legacy_z,
delta_collision_geom_center_legacy_x,delta_collision_geom_center_legacy_y,delta_collision_geom_center_legacy_z
```

Frame meanings:

- `W`: MuJoCo world frame.
- `B`: yaw-aligned reduced-body COM frame used by the controller.
- `legacy_nominal_B`: legacy nominal foot placement seed in frame `B`.
- `delta_*_legacy`: measured copied-state foot position minus `legacy_nominal_B`.

## Relevant Config

This probe explicitly activates the MIT humanoid config:

```text
config/mit_humanoid/my_controller.yaml
```

Important sections:

- `left_swing_hold_test.xml_path`: MuJoCo scene XML loaded by the probe.
- `model.xml_path`: fallback MuJoCo scene XML if the test-specific path is empty.
- `foot_placement.nominal_lateral_offset`: included in `legacy_nominal_B`.
- `model`: model path and gravity config used by the controller utilities.

## Notes

- This target is intentionally not added to CTest.
- This is a geometry/debug probe, not a closed-loop control test.
- The probe copies only keyframe `qpos`; it does not load keyframe qvel or ctrl.
