# Dashboard

This directory contains the graph-only telemetry dashboard for ConvexMPC.
It shows the live controller state, recent traces, and one to three main charts
from the shared-memory telemetry buffer.

The server entry point is `dashboard/app.py`, which serves:

- `dashboard/index.html`
- `dashboard/static/dashboard.css`
- `dashboard/static/dashboard.js`

## What It Does

- Displays the 12 controller state channels as time-series plots
- Shows the current robot, iteration, simulation time, and sequence number
- Lets you switch between all channels, pose channels, and motion channels
- Lets you toggle angle display between `deg` and `rad` with `deg` as the default
- Lets you stack up to three main canvases and switch each one with Prev/Next
- Lets each main canvas switch between raw, mean, and moving-average display modes
- Opens in a browser and refreshes automatically while the controller is running

The dashboard is intentionally limited to graphs and status. It does not include a
browser 3D display.

## Run

The usual entry point is the launcher script:

```bash
./scripts/launch_convexmpc.sh m n
```

That starts the controller and launches the dashboard on the first free port at or above
`8001` unless `CONVEXMPC_DASHBOARD_PORT` is set.

To run the dashboard directly from the active `convexmpc` conda environment:

```bash
python dashboard/app.py --host 127.0.0.1 --port 8001
```

Useful flags:

- `--host`: bind address, default `127.0.0.1`
- `--port`: HTTP port, default `8000`
- `--shared-memory`: state buffer name, default `convexmpc_dashboard_state`
- `--no-browser`: start the server without opening a browser window

## Environment

The dashboard reads these environment variables when present:

- `CONVEXMPC_SHM_NAME`
- `CONVEXMPC_DASHBOARD_HOST`
- `CONVEXMPC_DASHBOARD_PORT`
- `CONVEXMPC_DASHBOARD_OPEN_BROWSER`

The dashboard itself only needs the standard Python runtime for graph mode.

## Page Layout

- Header with live connection and controller status
- Toolbar with view, time-window, angle-unit, and main-panel count controls
- One to three main charts stacked vertically
- Trace grid for all channels in the current view

## Notes

- The launcher still accepts the robot selector and native MuJoCo window flag for
  the simulation binary.
- The dashboard uses the controller telemetry shared memory buffer only.
- The Python entry point is meant to run from the active `convexmpc` conda
  environment with `python dashboard/app.py ...`.
