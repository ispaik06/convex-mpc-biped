# Standing Debug Probes

These probes consume `logs/debug/standing_mpc/standing_mpc_debug_*.json`.

## Capturing A Log

Run the normal app and press `l` after the robot is running. The controller does
not solve MPC early; it writes one debug log after the next scheduled standing
MPC solve. Press `l` again later to capture another solve.

To queue logs by simulation time instead of pressing `l`, set
`logging.standing_mpc_debug_trigger_times` in `config/my_controller.yaml`, for
example `[1.5, 3.0]`. Each time queues one log when the simulation reaches that
time, and the log is still written after the next scheduled standing MPC solve.
The JSON metadata records `debug_request_source`, `debug_request_time`, and
`debug_trigger_time`.

`mpc.contact_wrench_model` in `config/my_controller.yaml` selects the MPC
contact wrench assumption. `full_wrench` leaves all 12 wrench inputs free.
`no_roll_moment` keeps the 12-input QP shape but adds equality constraints
`x_F^T M_W = 0` for each stance foot, where `x_F` is that foot/end-effector
local x axis expressed in world.

## SRB Reconstruction

```sh
python test/standing_debug/srb_reconstruct.py
python test/standing_debug/srb_reconstruct.py logs/debug/standing_mpc/standing_mpc_debug_YYYYMMDD_HHMMSS.json
```

The script recomputes `X = A_qp * x0 + B_qp * wrench`, checks it against the
logged predicted state horizon, and saves a trajectory plot to
`logs/debug/standing_mpc/plots/srb_reconstruct_YYYYMMDD_HHMMSS.png`.

## Contact Probe

```sh
cmake --build build --target stand_contact_probe
./build/test/standing_debug/stand_contact_probe
./build/test/standing_debug/stand_contact_probe logs/debug/standing_mpc/standing_mpc_debug_YYYYMMDD_HHMMSS.json
```

The probe restores `full_qpos`, `full_qvel`, applies `full_tau_command`, runs
`mj_forward`, and reports contact wrenches about both the foot contact site and
the foot link COM. It also saves the same concise text report to
`logs/debug/standing_mpc/contact_probe/txt/stand_contact_probe_YYYYMMDD_HHMMSS.txt`.
It saves a plot CSV to
`logs/debug/standing_mpc/contact_probe/csv/stand_contact_probe_YYYYMMDD_HHMMSS.csv`
and automatically calls
`python test/standing_debug/plot_contact_probe.py` to save a PNG to
`logs/debug/standing_mpc/contact_probe/plots/stand_contact_probe_YYYYMMDD_HHMMSS.png`.

The PNG compares the QP desired wrench with the MuJoCo-realized contact wrench
after applying the logged `full_tau_command`. Moment comparison uses the
reference point recorded in the standing MPC log: `foot_site` when the
controller used site end-effectors, or `foot_link_com` when it used body COMs.

## SRB Receding Horizon Probe

```sh
cmake --build build --target stand_rh_probe
./build/test/standing_debug/stand_rh_probe
./build/test/standing_debug/stand_rh_probe -n 80 logs/debug/standing_mpc/standing_mpc_debug_YYYYMMDD_HHMMSS.json
```

This is an internal SRB model test only. It reads the logged `x0`, fixed
standing reference, desired foot positions, foot local x axes, and logged
reduced-body mass and inertia. Then every rollout step rebuilds the standing
reference trajectory, gait constraints, SRB formulation, and QP. The first
predicted state from that solve becomes the next rollout state.

Outputs are saved under
`logs/debug/standing_mpc/receding_horizon/`:

- `txt/stand_rh_YYYYMMDD_HHMMSS.txt`: concise summary and first-solve mismatch
  against the source log.
- `csv/stand_rh_YYYYMMDD_HHMMSS.csv`: row-wise rollout data with all RPY,
  position XYZ, angular velocity XYZ, linear velocity XYZ, and all wrench
  components.
- `plots/stand_rh_YYYYMMDD_HHMMSS/`: run-specific plot folder.
- `plots/stand_rh_YYYYMMDD_HHMMSS/states.png`: state vs fixed reference.
- `plots/stand_rh_YYYYMMDD_HHMMSS/wrench.png`: first applied wrench from each
  receding-horizon solve.
- `plots/stand_rh_YYYYMMDD_HHMMSS/metrics.png`: state/input norms for quick
  instability checks.

The first-solve mismatch in the text report compares the rebuilt solve against
the source log while using the current `config/my_controller.yaml`; if weights
or constraints changed after the log was captured, this comparison will not be
zero.

The wrapper script can include this probe:

```sh
./scripts/run_standing_debug.sh -r -n 80
./scripts/run_standing_debug.sh -l logs/debug/standing_mpc/standing_mpc_debug_YYYYMMDD_HHMMSS.json -r -n 80
```
