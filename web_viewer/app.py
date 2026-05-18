#!/usr/bin/env python3
"""Minimal fullscreen MuJoCo WASM viewer server.

The server is intentionally separate from the telemetry dashboard. It serves the
viewer files, MIT humanoid model assets, and a small qpos stream backed by the
existing ConvexMPC POSIX shared-memory buffer.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import mimetypes
import os
import struct
import sys
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse


PROJECT_ROOT = Path(__file__).resolve().parents[1]
VIEWER_ROOT = Path(__file__).resolve().parent
MODEL_ROOT = PROJECT_ROOT / "models" / "mit_humanoid"
DEFAULT_QPOS_SHM_NAME = os.environ.get("CONVEXMPC_QPOS_SHM_NAME", "convexmpc_dashboard_qpos")

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


class QposSharedMemoryReader:
    def __init__(self, name: str) -> None:
        self.name = name.lstrip("/") or DEFAULT_QPOS_SHM_NAME
        self._posix_name = f"/{self.name}"
        self._fd = -1
        self._addr: int | None = None

    def _connect(self) -> memoryview:
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

    def snapshot(self) -> dict:
        try:
            view = self._connect()
            for _ in range(4):
                seq0 = struct.unpack_from("<Q", view, 0)[0]
                if seq0 % 2:
                    time.sleep(0.001)
                    continue
                unpacked = QPOS_STRUCT.unpack(view)
                seq1 = struct.unpack_from("<Q", view, 0)[0]
                if seq0 == seq1 and seq1 % 2 == 0:
                    qpos_dim = min(int(unpacked[6]), QPOS_CAPACITY)
                    robot_name = unpacked[3].split(b"\0", 1)[0].decode("utf-8", "replace")
                    model_xml_path = unpacked[4].split(b"\0", 1)[0].decode("utf-8", "replace")
                    return {
                        "ok": True,
                        "sequence": int(unpacked[0]),
                        "iteration": int(unpacked[1]),
                        "sim_time": float(unpacked[2]),
                        "robot_name": robot_name,
                        "model_xml_path": model_xml_path,
                        "qpos_dim": qpos_dim,
                        "qpos": list(unpacked[7 : 7 + qpos_dim]),
                    }
            raise RuntimeError("qpos shared memory was being written")
        except FileNotFoundError:
            self.close()
            return {
                "ok": False,
                "message": f"qpos shared memory '{self.name}' is not available",
            }
        except Exception as exc:  # Keep the viewer alive while main_zero restarts.
            self.close()
            return {"ok": False, "message": str(exc)}


def safe_relative_path(raw_path: str, root: Path) -> Path | None:
    relative = unquote(raw_path).lstrip("/")
    candidate = (root / relative).resolve()
    try:
        candidate.relative_to(root.resolve())
    except ValueError:
        return None
    return candidate


def mit_model_manifest() -> list[dict]:
    files: list[dict] = []
    for path in sorted(MODEL_ROOT.rglob("*")):
        if not path.is_file() or any(part.startswith(".") for part in path.relative_to(MODEL_ROOT).parts):
            continue
        relative = path.relative_to(MODEL_ROOT).as_posix()
        if path.suffix.lower() not in {".xml", ".stl"}:
            continue
        files.append(
            {
                "path": relative,
                "url": f"/model/mit_humanoid/{relative}",
                "binary": path.suffix.lower() != ".xml",
            }
        )
    return files


class ViewerHandler(SimpleHTTPRequestHandler):
    server_version = "ConvexMPCViewer/0.1"

    def log_message(self, format: str, *args) -> None:
        print(f"[web_viewer] {self.address_string()} - {format % args}")

    @property
    def qpos_reader(self) -> QposSharedMemoryReader:
        return self.server.qpos_reader  # type: ignore[attr-defined]

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/":
            self.serve_file(VIEWER_ROOT / "index.html")
            return
        if path == "/api/model-files":
            self.send_json({"root": "/working", "scene": "scene.xml", "files": mit_model_manifest()})
            return
        if path == "/api/qpos":
            self.send_json(self.qpos_reader.snapshot())
            return
        if path == "/events/qpos":
            self.stream_qpos()
            return
        if path.startswith("/model/mit_humanoid/"):
            rel = path.removeprefix("/model/mit_humanoid/")
            file_path = safe_relative_path(rel, MODEL_ROOT)
            if file_path is None:
                self.send_error(HTTPStatus.FORBIDDEN)
                return
            self.serve_file(file_path)
            return
        if path.startswith("/static/"):
            file_path = safe_relative_path(path.removeprefix("/static/"), VIEWER_ROOT / "static")
            if file_path is None:
                self.send_error(HTTPStatus.FORBIDDEN)
                return
            self.serve_file(file_path)
            return
        if path.startswith("/node_modules/"):
            file_path = safe_relative_path(path.removeprefix("/node_modules/"), VIEWER_ROOT / "node_modules")
            if file_path is None:
                self.send_error(HTTPStatus.FORBIDDEN)
                return
            self.serve_file(file_path)
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def send_json(self, data: dict) -> None:
        payload = json.dumps(data, separators=(",", ":")).encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def stream_qpos(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        while True:
            payload = json.dumps(self.qpos_reader.snapshot(), separators=(",", ":"))
            try:
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return
            time.sleep(1.0 / 60.0)

    def serve_file(self, file_path: Path) -> None:
        if not file_path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        content_type = mimetypes.guess_type(file_path.name)[0] or "application/octet-stream"
        if file_path.suffix == ".js":
            content_type = "application/javascript; charset=utf-8"
        elif file_path.suffix == ".css":
            content_type = "text/css; charset=utf-8"
        elif file_path.suffix == ".wasm":
            content_type = "application/wasm"
        data = file_path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


class ViewerServer(ThreadingHTTPServer):
    def handle_error(self, request, client_address) -> None:
        exc = sys.exc_info()[1]
        if isinstance(exc, (BrokenPipeError, ConnectionResetError)):
            return
        super().handle_error(request, client_address)


def main() -> int:
    parser = argparse.ArgumentParser(description="ConvexMPC MuJoCo WASM viewer")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=int(os.environ.get("CONVEXMPC_WEB_VIEWER_PORT", "8010")))
    parser.add_argument("--qpos-shm", default=DEFAULT_QPOS_SHM_NAME)
    args = parser.parse_args()

    qpos_reader = QposSharedMemoryReader(args.qpos_shm)
    server = ViewerServer((args.host, args.port), ViewerHandler)
    server.qpos_reader = qpos_reader  # type: ignore[attr-defined]
    url = f"http://{args.host}:{args.port}"
    print(f"[web_viewer] listening on {url}")
    print(f"[web_viewer] qpos shared memory: {qpos_reader.name}")
    try:
        server.serve_forever()
    finally:
        qpos_reader.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
