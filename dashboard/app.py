#!/usr/bin/env python3

from __future__ import annotations

import argparse
import ctypes
import json
import mimetypes
import os
import struct
import sys
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from multiprocessing import shared_memory
from pathlib import Path
from urllib.parse import unquote, urlparse


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
    "cmd_vel_x",
    "cmd_vel_y",
    "cmd_psi_dot",
    "target_roll",
    "target_pitch",
    "target_pos_z",
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

LAYOUT_VERSION = 2
LAYOUT_VERSION = 3
LAYOUT = struct.Struct("<QQd32s18dIIQQ")
LAYOUT_SIZE = LAYOUT.size
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8000
DEFAULT_SHM_NAME = os.environ.get("CONVEXMPC_SHM_NAME", "convexmpc_dashboard_state")
DEFAULT_QPOS_SHM_NAME = os.environ.get("CONVEXMPC_QPOS_SHM_NAME", "convexmpc_dashboard_qpos")
DEFAULT_WINDOW_SECONDS = 10.0
WINDOW_OPTIONS = (5, 10, 20, 30)
MAX_MAIN_PANELS = 3
MIN_MAIN_PANELS = 1

BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
WEB_VIEWER_DIR = PROJECT_ROOT / "web_viewer"
MODEL_ROOT = PROJECT_ROOT / "models" / "mit_humanoid"
INDEX_PATH = BASE_DIR / "index.html"
STATIC_DIR = BASE_DIR / "static"
STATIC_PATHS = {
    "/static/dashboard.css": STATIC_DIR / "dashboard.css",
    "/static/dashboard.js": STATIC_DIR / "dashboard.js",
    "/static/viewer.css": WEB_VIEWER_DIR / "static" / "viewer.css",
    "/static/viewer.js": WEB_VIEWER_DIR / "static" / "viewer.js",
}

QPOS_CAPACITY = 256
QPOS_STRUCT = struct.Struct("<QQd32s256sII" + ("d" * QPOS_CAPACITY) + "QQ")
QPOS_LAYOUT_SIZE = QPOS_STRUCT.size
LIBC = ctypes.CDLL(None, use_errno=True)
LIBC.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_uint]
LIBC.shm_open.restype = ctypes.c_int
LIBC.mmap.argtypes = [
    ctypes.c_void_p,
    ctypes.c_size_t,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_int,
    ctypes.c_long,
]
LIBC.mmap.restype = ctypes.c_void_p
LIBC.munmap.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
LIBC.munmap.restype = ctypes.c_int
MAP_FAILED = ctypes.c_void_p(-1).value
O_RDONLY = 0
PROT_READ = 0x01
MAP_SHARED = 0x0001


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
        state = list(fields[4:22])
        version = fields[22]
        state_dim = fields[23]
        robot_name = robot_raw.split(b"\x00", 1)[0].decode("utf-8", "replace")
        status = "live" if sequence > 0 else "priming"
        message = "live data" if sequence > 0 else "waiting for first controller sample"
        if version != LAYOUT_VERSION:
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


class QposSharedMemoryClient:
    def __init__(self, shared_memory_name: str):
        self.shared_memory_name = shared_memory_name.lstrip("/") or DEFAULT_QPOS_SHM_NAME
        self._posix_name = f"/{self.shared_memory_name}"
        self._fd = -1
        self._addr: int | None = None

    def _open(self) -> memoryview:
        if self._addr is None:
            self._fd = LIBC.shm_open(self._posix_name.encode("utf-8"), O_RDONLY, 0)
            if self._fd < 0:
                err = ctypes.get_errno()
                raise FileNotFoundError(os.strerror(err))
            addr = LIBC.mmap(None, QPOS_LAYOUT_SIZE, PROT_READ, MAP_SHARED, self._fd, 0)
            os.close(self._fd)
            self._fd = -1
            if addr == MAP_FAILED:
                err = ctypes.get_errno()
                raise OSError(err, os.strerror(err))
            self._addr = int(addr)

        array_type = ctypes.c_ubyte * QPOS_LAYOUT_SIZE
        return memoryview(array_type.from_address(self._addr))

    def close(self) -> None:
        if self._addr is not None:
            LIBC.munmap(ctypes.c_void_p(self._addr), QPOS_LAYOUT_SIZE)
            self._addr = None
        if self._fd >= 0:
            os.close(self._fd)
            self._fd = -1

    def snapshot(self) -> dict[str, object]:
        try:
            buf = self._open()
            for _ in range(8):
                seq1 = struct.unpack_from("<Q", buf, 0)[0]
                if seq1 % 2 == 1:
                    time.sleep(0.001)
                    continue
                fields = QPOS_STRUCT.unpack(buf)
                seq2 = struct.unpack_from("<Q", buf, 0)[0]
                if seq1 == seq2 and seq2 % 2 == 0:
                    qpos_dim = min(int(fields[6]), QPOS_CAPACITY)
                    robot_name = fields[3].split(b"\x00", 1)[0].decode("utf-8", "replace")
                    model_xml_path = fields[4].split(b"\x00", 1)[0].decode("utf-8", "replace")
                    return {
                        "ok": True,
                        "sequence": int(fields[0]),
                        "iteration": int(fields[1]),
                        "sim_time": float(fields[2]),
                        "robot_name": robot_name,
                        "model_xml_path": model_xml_path,
                        "qpos_dim": qpos_dim,
                        "qpos": list(fields[7 : 7 + qpos_dim]),
                    }
            raise RuntimeError("qpos shared memory was being written")
        except FileNotFoundError:
            self.close()
            return {
                "ok": False,
                "message": f"qpos shared memory '{self.shared_memory_name}' is not available",
            }
        except Exception as exc:
            self.close()
            return {"ok": False, "message": str(exc)}


def _safe_relative_path(raw_path: str, root: Path) -> Path | None:
    relative = unquote(raw_path).lstrip("/")
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root.resolve())
    except ValueError:
        return None
    return candidate


def _mit_model_manifest() -> list[dict[str, object]]:
    files: list[dict[str, object]] = []
    for path in sorted(MODEL_ROOT.rglob("*")):
        if not path.is_file() or any(part.startswith(".") for part in path.relative_to(MODEL_ROOT).parts):
            continue
        if path.suffix.lower() not in {".xml", ".stl"}:
            continue
        relative = path.relative_to(MODEL_ROOT).as_posix()
        files.append(
            {
                "path": relative,
                "url": f"/model/mit_humanoid/{relative}",
                "binary": path.suffix.lower() != ".xml",
            }
        )
    return files


class DashboardHTTPServer(ThreadingHTTPServer):
    def __init__(
        self,
        server_address,
        RequestHandlerClass,
        state_reader: DashboardSharedMemoryClient,
        qpos_reader: QposSharedMemoryClient,
    ):
        super().__init__(server_address, RequestHandlerClass)
        self.state_reader = state_reader
        self.qpos_reader = qpos_reader

    def handle_error(self, request, client_address) -> None:
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError)):
            return
        super().handle_error(request, client_address)


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

    def _serve_file_auto_type(self, path: Path) -> None:
        content_type = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        if path.suffix == ".js":
            content_type = "application/javascript; charset=utf-8"
        elif path.suffix == ".css":
            content_type = "text/css; charset=utf-8"
        elif path.suffix == ".wasm":
            content_type = "application/wasm"
        self._send_file(path, content_type)

    def _snapshot_payload(self) -> dict[str, object]:
        payload = self.server.state_reader.snapshot()
        payload["server_time"] = time.time()
        return payload

    def do_GET(self) -> None:  # noqa: N802
        request_path = urlparse(self.path).path

        if request_path in {"/", "/index.html"}:
            self._send_bytes(_render_index_html(), "text/html; charset=utf-8")
            return

        asset_path = STATIC_PATHS.get(request_path)
        if asset_path is not None:
            self._serve_file_auto_type(asset_path)
            return

        if request_path == "/api/state":
            payload = self._snapshot_payload()
            self._send_bytes(
                json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return

        if request_path == "/api/model-files":
            payload = {"root": "/working", "scene": "scene.xml", "files": _mit_model_manifest()}
            self._send_bytes(
                json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return

        if request_path == "/api/qpos":
            self._send_bytes(
                json.dumps(self.server.qpos_reader.snapshot(), separators=(",", ":"), ensure_ascii=False).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return

        if request_path == "/events/qpos":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            while True:
                payload = json.dumps(self.server.qpos_reader.snapshot(), separators=(",", ":"), ensure_ascii=False)
                try:
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError):
                    return
                time.sleep(1.0 / 60.0)

        if request_path.startswith("/model/mit_humanoid/"):
            model_path = _safe_relative_path(request_path.removeprefix("/model/mit_humanoid/"), MODEL_ROOT)
            if model_path is None:
                self.send_error(403, "Forbidden")
                return
            self._serve_file_auto_type(model_path)
            return

        if request_path.startswith("/node_modules/"):
            module_path = _safe_relative_path(request_path.removeprefix("/node_modules/"), WEB_VIEWER_DIR / "node_modules")
            if module_path is None:
                self.send_error(403, "Forbidden")
                return
            self._serve_file_auto_type(module_path)
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
        "--qpos-shm",
        default=os.environ.get("CONVEXMPC_QPOS_SHM_NAME", DEFAULT_QPOS_SHM_NAME),
        dest="qpos_shm",
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
    qpos_reader = QposSharedMemoryClient(args.qpos_shm)
    server = DashboardHTTPServer((args.host, args.port), DashboardHandler, reader, qpos_reader)
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
    print(f"[dashboard] qpos shared memory: {args.qpos_shm}")

    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        reader.close()
        qpos_reader.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
