# MPC Debug Probes

These tools consume `logs/debug/standing_mpc/standing_mpc_debug_*.json` and `logs/debug/walking_mpc/walking_mpc_debug_*.json`.

## What the Log Contains

The JSON stores a `controller_config` snapshot together with the data needed to replay and verify one MPC solve in either standing or walking mode.

- `controller_config`: the parsed controller parameters used for the solve, including timing, model, MPC weight diagonals, swing settings, foot placement, logging, and test defaults
- `metadata.robot_type`: the robot type label used by the plot and report titles
- `metadata`: `locomotion_mode`, `horizon_clock_t0`, `debug_request_source`, `debug_request_time`, `debug_trigger_time`, `horizon_steps`, `dt_mpc`, `foot_end_effector_source`, `desired_wrench_reference_point`, `contact_wrench_model`
- `user_command`: keyboard command values active when the MPC solve was logged
- `model`: reduced-body mass, gravity, COM offset, and reduced-body inertia
- `initial_state`: `x0`, torso pose, and torso velocities
- `robot_state`: full `qpos`, `qvel`, and the applied torque command
- `feet`: desired foot positions and per-foot kinematics from the logged solve
- `reference_trajectory`: `tk`, `psi`, foot paths, and `X_ref_by_step`
- `formulation`: `A_c`, `B_c`, `inertia_W`, `A_qp`, `B_qp`
- `solution`: wrench horizon and predicted state horizon
- `wrench_to_torque`: Jacobian-transpose mapping from MPC wrench to packed leg torque, using standing combined Jacobians when available and per-leg Jacobians otherwise
- `standing_wrench_to_torque`: legacy alias for the same mapping section

The probes use the logged `controller_config` snapshot when it is present. Older logs without that field fall back to the current `config/my_controller.yaml`.

## Capturing A Log

Run the normal app and press `Shift+L` after the robot is running. The controller does not solve MPC early; it writes one debug log after the next scheduled MPC solve. Press `Shift+L` again later to capture another solve.

Standing captures are saved under `logs/debug/standing_mpc/standing_mpc_debug_YYYYMMDD_HHMMSS.json`; walking captures are saved under `logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json`.

To queue logs by simulation time instead of pressing `Shift+L`, set `logging.standing_mpc_debug_trigger_times` in `config/my_controller.yaml`, for example `[1.5, 3.0]`. Each time queues one log when the simulation reaches that time, and the log is still written after the next scheduled MPC solve.

## SRB Reconstruction

```sh
python test/standing_debug/srb_reconstruct.py
python test/standing_debug/srb_reconstruct.py logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json
```

This script is a replay and consistency check, not a fresh MPC solve. It reads the logged `x0`, `A_qp`, `B_qp`, and wrench horizon, recomputes `A_qp * x0 + B_qp * wrench`, checks that against the stored predicted state horizon, and saves a trajectory plot next to the log under `logs/debug/{standing_mpc,walking_mpc}/plots/`.

Use this when you want to verify that the logged first solve is self-consistent.

## Contact Probe

```sh
cmake --build build --target stand_contact_probe
./build/test/standing_debug/stand_contact_probe
./build/test/standing_debug/stand_contact_probe logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json
```

This probe restores `full_qpos`, `full_qvel`, applies `full_tau_command`, runs `mj_forward`, and reports contact wrenches about both the foot contact site and the foot collision-geom center. It also writes a Markdown report, a CSV, and a PNG under `logs/debug/{standing_mpc,walking_mpc}/contact_probe/` based on the log mode.

The PNG compares the QP desired wrench with the MuJoCo-realized contact wrench after applying the logged `full_tau_command`. Moment comparison uses the reference point recorded in the standing MPC log: `foot_site` when the controller used site end-effectors, or `foot_collision_geom_center` when it used collision-geom centers.

## Wrench Reconstruction

```sh
cmake --build build --target wrench_mapping_probe
python test/standing_debug/wrench_reconstruction.py
python test/standing_debug/wrench_reconstruction.py logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json
```

This analysis checks how much of the QP wrench survives the torque projection through the current foot Jacobians:

$$
tau_{qp} = A w_{qp}, \qquad \hat{w} = A^+ tau_{qp} = A^+ A w_{qp}
$$

where `A` is the logged `wrench_to_tau_jacobian`. For two 5-DoF legs, `A` is usually `10 x 12`, so the reconstructed wrench is the row-space projection of the QP wrench. The useful plot is `w_qp` versus `hat{w}`, with the per-component error below it.

If an older log does not contain `wrench_to_tau_jacobian`, the script calls `build/test/standing_debug/wrench_mapping_probe` to restore `full_qpos/full_qvel` in MuJoCo and rebuild `A` from the foot Jacobians.

Outputs are saved under `logs/debug/{standing_mpc,walking_mpc}/wrench_reconstruction/` based on the log mode:

- `reports/stand_wrench_reconstruction_YYYYMMDD_HHMMSS.md` or `reports/walk_wrench_reconstruction_YYYYMMDD_HHMMSS.md`: Markdown summary with equations, rank, singular values, and component errors.
- `csv/stand_wrench_reconstruction_YYYYMMDD_HHMMSS.csv` or `csv/walk_wrench_reconstruction_YYYYMMDD_HHMMSS.csv`: component-wise QP wrench, reconstructed wrench, and error.
- `plots/stand_wrench_reconstruction_YYYYMMDD_HHMMSS.png` or `plots/walk_wrench_reconstruction_YYYYMMDD_HHMMSS.png`: QP-vs-reconstructed wrench comparison.

## SRB Receding Horizon Probe

```sh
cmake --build build --target stand_rh_probe
./build/test/standing_debug/stand_rh_probe
./build/test/standing_debug/stand_rh_probe -n 80 logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json
./build/test/standing_debug/stand_rh_probe -n 120 --x-dot-final 0.7 logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json
```

This is the actual receding-horizon replay path. It reads the logged `x0`, logged command, initial reference, foot local x axes, and logged reduced-body mass and inertia. Then every rollout step rebuilds the reference trajectory, gait constraints, walking foot targets, SRB formulation, and QP before solving again. The first predicted state from that solve becomes the next rollout state.

For walking logs, the replay matches the online controller's receding-reference policy: each step
seeds planar position and yaw from the rollout state, while roll, pitch, and nominal height come
from the logged first reference. The logged body-height offset is applied again inside
`ReferenceTrajectory`, so the replay seed removes that offset before building the horizon.
The walking foot targets are seeded from the logged targets, then recomputed by `SwingFootPlanner`
as the replay phase advances; pass `--fixed-foot-points` only when you need the older fixed-anchor
behavior. Use `--x-dot-final MPS` for a linear raw-command ramp from the logged `x_dot`, or
`--x-dot-rate MPS2` with an optional `--x-dot-final MPS` cap. The CSV records the filtered command
used by the MPC, the gait phase/contact state, and the desired foot targets for every row.

The first-solve mismatch in the Markdown report compares the rebuilt solve against the source log while using the current replay policy and the logged `controller_config` snapshot when available. If weights, constraints, command schedule, contact phase sampling, or foot-target policy differ from the captured solve, this comparison will not be zero.

Outputs are saved under `logs/debug/{standing_mpc,walking_mpc}/receding_horizon/` based on the log mode:

- `reports/stand_rh_YYYYMMDD_HHMMSS.md` or `reports/walk_rh_YYYYMMDD_HHMMSS.md`: Markdown summary and first-solve mismatch against the source log.
- `csv/stand_rh_YYYYMMDD_HHMMSS.csv` or `csv/walk_rh_YYYYMMDD_HHMMSS.csv`: row-wise rollout data with all RPY, position XYZ, angular velocity XYZ, linear velocity XYZ, and all wrench components.
- `plots/stand_rh_YYYYMMDD_HHMMSS/` or `plots/walk_rh_YYYYMMDD_HHMMSS/`: run-specific plot folder.
- `states.png`: state vs reference.
- `wrench.png`: first applied wrench from each receding-horizon solve.
- `metrics.png`: state/input norms for quick instability checks.

## Wrapper Scripts

The general wrapper runs the MPC horizon plot, contact probe, wrench reconstruction, and receding-horizon probe for one selected log. You must choose exactly one source selector:

```sh
./scripts/run_mpc_debug.sh --standing -n 80
./scripts/run_mpc_debug.sh --walking -n 80
./scripts/run_mpc_debug.sh --all-latest -n 80
./scripts/run_mpc_debug.sh -l logs/debug/walking_mpc/walking_mpc_debug_YYYYMMDD_HHMMSS.json -n 80
./scripts/run_mpc_debug.sh --walking -n 120 --x-dot-final 0.7
```

The wrapper captures each analysis' detailed stdout/stderr and prints only a per-analysis status summary. `[OK]` means the expected artifacts were generated, `[SKIP]` means the analysis wrote a report explaining why a plot/CSV was not possible for that log, and `[FAIL]` means the command failed or an expected artifact was missing.
