#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import struct
import sys
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from multiprocessing import shared_memory
from pathlib import Path


STATE_LABELS = (
    "roll",
    "pitch",
    "yaw",
    "pos_x",
    "pos_y",
    "pos_z",
    "omega_x",
    "omega_y",
    "omega_z",
    "vel_x",
    "vel_y",
    "vel_z",
)


def _chart(
    label: str,
    title: str,
    group: str,
    group_label: str,
    unit: str,
    color: str,
    *,
    scale: str = "auto",
    min_span: float = 1.0,
    precision: int = 3,
) -> dict[str, object]:
    return {
        "label": label,
        "title": title,
        "group": group,
        "group_label": group_label,
        "unit": unit,
        "color": color,
        "scale": scale,
        "min_span": min_span,
        "precision": precision,
    }


CHART_CONFIGS = (
    _chart("roll", "Roll", "pose", "Pose", "rad", "#7dd3fc", scale="symmetric", min_span=0.8),
    _chart("pitch", "Pitch", "pose", "Pose", "rad", "#4ade80", scale="symmetric", min_span=0.8),
    _chart("yaw", "Yaw", "pose", "Pose", "rad", "#f59e0b", scale="symmetric", min_span=1.0),
    _chart("pos_x", "Pos X", "pose", "Pose", "m", "#22c55e", scale="auto", min_span=0.5),
    _chart("pos_y", "Pos Y", "pose", "Pose", "m", "#f97316", scale="auto", min_span=0.5),
    _chart("pos_z", "Pos Z", "pose", "Pose", "m", "#60a5fa", scale="auto", min_span=0.4),
    _chart("omega_x", "Omega X", "motion", "Motion", "rad/s", "#f43f5e", scale="symmetric", min_span=0.8),
    _chart("omega_y", "Omega Y", "motion", "Motion", "rad/s", "#14b8a6", scale="symmetric", min_span=0.8),
    _chart("omega_z", "Omega Z", "motion", "Motion", "rad/s", "#eab308", scale="symmetric", min_span=0.8),
    _chart("vel_x", "Vel X", "motion", "Motion", "m/s", "#34d399", scale="symmetric", min_span=1.0),
    _chart("vel_y", "Vel Y", "motion", "Motion", "m/s", "#fb7185", scale="symmetric", min_span=1.0),
    _chart("vel_z", "Vel Z", "motion", "Motion", "m/s", "#38bdf8", scale="symmetric", min_span=1.0),
)

LAYOUT = struct.Struct("<QQd32s12dIIQQ")
LAYOUT_SIZE = LAYOUT.size
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8000
DEFAULT_SHM_NAME = os.environ.get("CONVEXMPC_SHM_NAME", "convexmpc_dashboard_state")
DEFAULT_WINDOW_SECONDS = 10.0
WINDOW_OPTIONS = (5, 10, 20, 30)
MAX_MAIN_PANELS = 3
MIN_MAIN_PANELS = 1

BASE_DIR = Path(__file__).resolve().parent
INDEX_PATH = BASE_DIR / "index.html"
STATIC_DIR = BASE_DIR / "static"
STATIC_PATHS = {
    "/static/dashboard.css": STATIC_DIR / "dashboard.css",
    "/static/dashboard.js": STATIC_DIR / "dashboard.js",
}


def _read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _render_index_html() -> bytes:
    html = _read_text(INDEX_PATH)
    html = html.replace("%CHART_CONFIG%", json.dumps(CHART_CONFIGS, ensure_ascii=False))
    html = html.replace("%STATE_LABELS%", json.dumps(list(STATE_LABELS)))
    html = html.replace("%DEFAULT_WINDOW_SECONDS%", json.dumps(DEFAULT_WINDOW_SECONDS))
    html = html.replace("%WINDOW_OPTIONS%", json.dumps(list(WINDOW_OPTIONS)))
    html = html.replace("%MAX_MAIN_PANELS%", json.dumps(MAX_MAIN_PANELS))
    html = html.replace("%MIN_MAIN_PANELS%", json.dumps(MIN_MAIN_PANELS))
    return html.encode("utf-8")


class DashboardSharedMemoryClient:
    def __init__(self, shared_memory_name: str):
        self.shared_memory_name = shared_memory_name.lstrip("/") or "convexmpc_dashboard_state"
        self._shm: shared_memory.SharedMemory | None = None
        self._last_error: str | None = None

    @property
    def is_open(self) -> bool:
        return self._shm is not None

    def close(self) -> None:
        if self._shm is not None:
            self._shm.close()
            self._shm = None

    def _open(self) -> bool:
        if self._shm is not None:
            return True
        try:
            self._shm = shared_memory.SharedMemory(
                name=self.shared_memory_name,
                create=False,
                track=False,
            )
        except TypeError:
            self._shm = shared_memory.SharedMemory(
                name=self.shared_memory_name,
                create=False,
            )
        except FileNotFoundError:
            self._last_error = "shared memory not available yet"
            return False
        except OSError as exc:
            self._last_error = str(exc)
            return False

        if self._shm.size < LAYOUT_SIZE:
            self._last_error = (
                f"shared memory segment is too small: {self._shm.size} < {LAYOUT_SIZE}"
            )
            self.close()
            return False

        self._last_error = None
        return True

    def snapshot(self) -> dict[str, object]:
        if not self._open():
            return {
                "connected": False,
                "status": "waiting",
                "message": self._last_error or "waiting for controller",
                "shared_memory_name": self.shared_memory_name,
                "robot_name": "",
                "sequence": 0,
                "iteration": 0,
                "sim_time": 0.0,
                "version": None,
                "state_dim": None,
                "state": [],
            }

        assert self._shm is not None
        buf = self._shm.buf
        seq1 = 0
        seq2 = 0
        fields = None
        for _ in range(8):
            seq1 = struct.unpack_from("<Q", buf, 0)[0]
            if seq1 % 2 == 1:
                time.sleep(0.001)
                continue
            fields = LAYOUT.unpack_from(buf, 0)
            seq2 = struct.unpack_from("<Q", buf, 0)[0]
            if seq1 == seq2 and seq2 % 2 == 0:
                break
            fields = None
        if fields is None:
            return {
                "connected": True,
                "status": "busy",
                "message": "controller is updating the shared buffer",
                "shared_memory_name": self.shared_memory_name,
                "robot_name": "",
                "sequence": seq2,
                "iteration": 0,
                "sim_time": 0.0,
                "version": None,
                "state_dim": None,
                "state": [],
            }

        sequence, iteration, sim_time, robot_raw = fields[:4]
        state = list(fields[4:16])
        version = fields[16]
        state_dim = fields[17]
        robot_name = robot_raw.split(b"\x00", 1)[0].decode("utf-8", "replace")
        status = "live" if sequence > 0 else "priming"
        message = "live data" if sequence > 0 else "waiting for first controller sample"
        if version != 1:
            status = "error"
            message = f"unexpected layout version {version}"
        elif state_dim != len(STATE_LABELS):
            status = "error"
            message = f"unexpected state dimension {state_dim}"

        return {
            "connected": True,
            "status": status,
            "message": message,
            "shared_memory_name": self.shared_memory_name,
            "robot_name": robot_name,
            "sequence": int(sequence),
            "iteration": int(iteration),
            "sim_time": float(sim_time),
            "version": int(version),
            "state_dim": int(state_dim),
            "state": state,
            "state_map": {label: state[idx] for idx, label in enumerate(STATE_LABELS)},
        }


class DashboardHTTPServer(ThreadingHTTPServer):
    def __init__(self, server_address, RequestHandlerClass, state_reader: DashboardSharedMemoryClient):
        super().__init__(server_address, RequestHandlerClass)
        self.state_reader = state_reader


class DashboardHandler(BaseHTTPRequestHandler):
    def _send_bytes(self, content: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _send_file(self, path: Path, content_type: str) -> None:
        if not path.is_file():
            self.send_error(404, "Not Found")
            return
        self._send_bytes(path.read_bytes(), content_type)

    def _snapshot_payload(self) -> dict[str, object]:
        payload = self.server.state_reader.snapshot()
        payload["server_time"] = time.time()
        return payload

    def do_GET(self) -> None:  # noqa: N802
        request_path = self.path.split("?", 1)[0]

        if request_path in {"/", "/index.html"}:
            self._send_bytes(_render_index_html(), "text/html; charset=utf-8")
            return

        asset_path = STATIC_PATHS.get(request_path)
        if asset_path is not None:
            content_type = "text/css; charset=utf-8" if asset_path.suffix == ".css" else "text/javascript; charset=utf-8"
            self._send_file(asset_path, content_type)
            return

        if request_path == "/api/state":
            payload = self._snapshot_payload()
            self._send_bytes(
                json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return

        if request_path == "/api/health":
            self._send_bytes(b"ok", "text/plain; charset=utf-8")
            return

        self.send_error(404, "Not Found")

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ConvexMPC dashboard")
    parser.add_argument("--host", default=os.environ.get("CONVEXMPC_DASHBOARD_HOST", DEFAULT_HOST))
    parser.add_argument(
        "--port",
        type=int,
        default=int(os.environ.get("CONVEXMPC_DASHBOARD_PORT", DEFAULT_PORT)),
    )
    parser.add_argument(
        "--shared-memory",
        default=os.environ.get("CONVEXMPC_SHM_NAME", DEFAULT_SHM_NAME),
        dest="shared_memory",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="Do not open the browser automatically",
    )
    args = parser.parse_args(argv)
    return args


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    reader = DashboardSharedMemoryClient(args.shared_memory)
    server = DashboardHTTPServer((args.host, args.port), DashboardHandler, reader)
    url = f"http://{args.host}:{args.port}"
    start_url = url

    if not args.no_browser and os.environ.get("CONVEXMPC_DASHBOARD_OPEN_BROWSER", "0") not in {
        "0",
        "false",
        "False",
        "",
    }:
        webbrowser.open(start_url, new=2)

    print(f"[dashboard] listening on {url}")
    print(f"[dashboard] shared memory: {args.shared_memory}")

    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        reader.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
