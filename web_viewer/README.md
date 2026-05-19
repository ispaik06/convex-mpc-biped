# ConvexMPC Web Viewer

This is the standalone MuJoCo WASM viewer server. The normal dashboard path now
embeds the same viewer above the telemetry plots.

For the integrated telemetry + viewer dashboard, see
[docs/web_dashboard.md](../docs/web_dashboard.md).

## Install

```bash
npm install
```

## Run Through The Launcher

From the repository root:

```bash
./scripts/launch_convexmpc.sh --no-dashboard --web-viewer m n
```

Open the URL printed by the launcher, usually:

```text
http://127.0.0.1:8010
```

For a zero-command spawn test:

```bash
cmake --build build --target main_zero -j
./scripts/launch_convexmpc.sh --zero-command --no-dashboard --web-viewer m n
```

The standalone server serves the MIT humanoid XML/STL assets, streams `qpos` from
the POSIX shared-memory buffer, and renders visual/collision MuJoCo geometry in a
fullscreen Three.js scene.
