# Standing Debug Probes

These probes consume `logs/debug/standing_mpc/standing_mpc_debug_*.json`.

## Capturing A Log

Run the normal app and press `l` after the robot is running. The controller does
not solve MPC early; it writes one debug log after the next scheduled standing
MPC solve. Press `l` again later to capture another solve.

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
`logs/debug/standing_mpc/contact_probe/stand_contact_probe_YYYYMMDD_HHMMSS.txt`.
