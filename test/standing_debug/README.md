# Standing Debug Probes

These tools consume `logs/debug/standing_mpc/standing_mpc_debug_*.json`.

## What the Log Contains

The JSON stores a `controller_config` snapshot together with the data needed to replay and verify one standing MPC solve.

- `controller_config`: the parsed controller parameters used for the solve, including timing, model, MPC weight diagonals, swing settings, foot placement, logging, and test defaults
- `metadata.robot_type`: the robot type label used by the plot and report titles
- `metadata`: `debug_request_source`, `debug_request_time`, `debug_trigger_time`, `horizon_steps`, `dt_mpc`, `foot_end_effector_source`, `desired_wrench_reference_point`, `contact_wrench_model`
- `model`: reduced-body mass, gravity, COM offset, and reduced-body inertia
- `initial_state`: `x0`, torso pose, and torso velocities
- `robot_state`: full `qpos`, `qvel`, and the applied torque command
- `feet`: desired foot positions and per-foot kinematics from the logged solve
- `reference_trajectory`: `tk`, `psi`, foot paths, and `X_ref_by_step`
- `formulation`: `A_c`, `B_c`, `inertia_W`, `A_qp`, `B_qp`
- `solution`: wrench horizon and predicted state horizon
- `standing_wrench_to_torque`: stance Jacobians and the logged wrench-to-torque mapping

The probes use the logged `controller_config` snapshot when it is present. Older logs without that field fall back to the current `config/my_controller.yaml`.

## Capturing A Log

Run the normal app and press `l` after the robot is running. The controller does not solve MPC early; it writes one debug log after the next scheduled standing MPC solve. Press `l` again later to capture another solve.

To queue logs by simulation time instead of pressing `l`, set `logging.standing_mpc_debug_trigger_times` in `config/my_controller.yaml`, for example `[1.5, 3.0]`. Each time queues one log when the simulation reaches that time, and the log is still written after the next scheduled standing MPC solve.

## SRB Reconstruction

```sh
python test/standing_debug/srb_reconstruct.py
python test/standing_debug/srb_reconstruct.py logs/debug/standing_mpc/standing_mpc_debug_YYYYMMDD_HHMMSS.json
```

This script is a replay and consistency check, not a fresh MPC solve. It reads the logged `x0`, `A_qp`, `B_qp`, and wrench horizon, recomputes `A_qp * x0 + B_qp * wrench`, checks that against the stored predicted state horizon, and saves a trajectory plot to `logs/debug/standing_mpc/plots/srb_reconstruct_YYYYMMDD_HHMMSS.png`.

Use this when you want to verify that the logged first solve is self-consistent.

## Contact Probe

```sh
cmake --build build --target stand_contact_probe
./build/test/standing_debug/stand_contact_probe
./build/test/standing_debug/stand_contact_probe logs/debug/standing_mpc/standing_mpc_debug_YYYYMMDD_HHMMSS.json
```

This probe restores `full_qpos`, `full_qvel`, applies `full_tau_command`, runs `mj_forward`, and reports contact wrenches about both the foot contact site and the foot collision-geom center. It also writes a text report, a CSV, and a PNG under `logs/debug/standing_mpc/contact_probe/`.

The PNG compares the QP desired wrench with the MuJoCo-realized contact wrench after applying the logged `full_tau_command`. Moment comparison uses the reference point recorded in the standing MPC log: `foot_site` when the controller used site end-effectors, or `foot_collision_geom_center` when it used collision-geom centers.

## SRB Receding Horizon Probe

```sh
cmake --build build --target stand_rh_probe
./build/test/standing_debug/stand_rh_probe
./build/test/standing_debug/stand_rh_probe -n 80 logs/debug/standing_mpc/standing_mpc_debug_YYYYMMDD_HHMMSS.json
```

This is the actual receding-horizon replay path. It reads the logged `x0`, fixed standing reference, desired foot positions, foot local x axes, and logged reduced-body mass and inertia. Then every rollout step rebuilds the standing reference trajectory, gait constraints, SRB formulation, and QP before solving again. The first predicted state from that solve becomes the next rollout state.

The first-solve mismatch in the text report compares the rebuilt solve against the source log while using the current `config/my_controller.yaml`. If weights or constraints changed after the log was captured, this comparison will not be zero.

Outputs are saved under `logs/debug/standing_mpc/receding_horizon/`:

- `txt/stand_rh_YYYYMMDD_HHMMSS.txt`: concise summary and first-solve mismatch against the source log.
- `csv/stand_rh_YYYYMMDD_HHMMSS.csv`: row-wise rollout data with all RPY, position XYZ, angular velocity XYZ, linear velocity XYZ, and all wrench components.
- `plots/stand_rh_YYYYMMDD_HHMMSS/`: run-specific plot folder.
- `plots/stand_rh_YYYYMMDD_HHMMSS/states.png`: state vs fixed reference.
- `plots/stand_rh_YYYYMMDD_HHMMSS/wrench.png`: first applied wrench from each receding-horizon solve.
- `plots/stand_rh_YYYYMMDD_HHMMSS/metrics.png`: state/input norms for quick instability checks.

## Wrapper Script

The wrapper script can include this probe:

```sh
./scripts/run_standing_debug.sh -r -n 80
./scripts/run_standing_debug.sh -l logs/debug/standing_mpc/standing_mpc_debug_YYYYMMDD_HHMMSS.json -r -n 80
```
