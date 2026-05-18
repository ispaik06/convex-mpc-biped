# ConvexMPC Web Viewer

This is a separate MuJoCo WASM viewer test, independent from `dashboard/`.
It focuses on fast fullscreen rendering of the MIT humanoid from the qpos shared
memory stream published by `apps/main_zero.cpp`.

## Run

```sh
cmake --build build --target main_zero
./build/apps/main_zero
```

In another terminal:

```sh
cd web_viewer
npm install
python3 app.py --host 127.0.0.1 --port 8010
```

Open `http://127.0.0.1:8010`.

By default, visual geoms (`group="1"`) and the floor (`group="2"`) are rendered.
Collision geoms (`group="3"`) and debug geoms (`group="4"`) are hidden to keep the
view light. Add `?collision=1` or `?debug=1` to show them.
