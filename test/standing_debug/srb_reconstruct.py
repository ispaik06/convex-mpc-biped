"""Reconstruct and plot the SRB horizon from a standing MPC debug JSON log."""

from __future__ import annotations

import json
import sys
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_LOG_DIR = PROJECT_ROOT / "logs" / "debug" / "standing_mpc"
DEFAULT_PLOT_DIR = DEFAULT_LOG_DIR / "plots"
STATE_DIM = 13
INPUT_DIM = 12


def latest_log_path() -> Path:
    paths = sorted(DEFAULT_LOG_DIR.glob("standing_mpc_debug_*.json"))
    if not paths:
        raise FileNotFoundError(f"no standing MPC debug logs found in {DEFAULT_LOG_DIR}")
    return paths[-1]


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def timestamp_token() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def default_plot_path() -> Path:
    return DEFAULT_PLOT_DIR / f"srb_reconstruct_{timestamp_token()}.png"


def matrix_like(value, name: str):
    if isinstance(value, dict) and {"rows", "cols", "data"}.issubset(value):
        rows = int(value["rows"])
        cols = int(value["cols"])
        data = value["data"]
        if data and all(isinstance(row, list) for row in data):
            if any(any(isinstance(item, list) for item in row) for row in data):
                raise ValueError(f"{name} is a matrix sequence, not a single matrix")
            matrix = [[float(item) for item in row] for row in data]
        else:
            flat = [float(item) for item in data]
            if len(flat) != rows * cols:
                raise ValueError(f"{name} data length does not match rows*cols")
            matrix = [flat[row * cols : (row + 1) * cols] for row in range(rows)]
        if len(matrix) != rows or any(len(row) != cols for row in matrix):
            raise ValueError(f"{name} shape does not match rows/cols metadata")
        return matrix

    if not isinstance(value, list) or not value or not all(isinstance(row, list) for row in value):
        raise ValueError(f"{name} is not matrix-like")

    cols = len(value[0])
    if any(len(row) != cols for row in value):
        raise ValueError(f"{name} has inconsistent row lengths")
    return [[float(item) for item in row] for row in value]


def vector_like(value, name: str):
    if isinstance(value, dict) and "data" in value:
        value = value["data"]
    if not isinstance(value, list):
        raise ValueError(f"{name} is not vector-like")

    if value and all(isinstance(row, list) for row in value):
        flat = [float(item) for row in value for item in row]
    else:
        flat = [float(item) for item in value]

    if not flat:
        raise ValueError(f"{name} is empty")
    return flat


def predicted_state_vector(log: dict):
    solution = log["solution"]
    if "predicted_state_horizon_vector" in solution:
        return vector_like(solution["predicted_state_horizon_vector"], "solution.predicted_state_horizon_vector")
    return flatten_matrix(matrix_like(solution["predicted_state_horizon"], "solution.predicted_state_horizon"))


def wrench_vector(log: dict):
    solution = log["solution"]
    if "wrench_horizon_vector" in solution:
        return vector_like(solution["wrench_horizon_vector"], "solution.wrench_horizon_vector")
    return flatten_matrix(matrix_like(solution["wrench_horizon"], "solution.wrench_horizon"))


def reconstruct(log: dict):
    x0 = vector_like(log["initial_state"]["x0"], "initial_state.x0")
    wrench = wrench_vector(log)
    a_qp = matrix_like(log["formulation"]["A_qp"], "formulation.A_qp")
    b_qp = matrix_like(log["formulation"]["B_qp"], "formulation.B_qp")

    if len(x0) != STATE_DIM:
        raise ValueError(f"x0 has size {len(x0)}, expected {STATE_DIM}")
    if len(wrench) % INPUT_DIM != 0:
        raise ValueError(f"wrench horizon has size {len(wrench)}, not divisible by {INPUT_DIM}")
    if not a_qp or len(a_qp[0]) != STATE_DIM:
        raise ValueError(f"A_qp shape is {matrix_shape(a_qp)}, expected second dimension {STATE_DIM}")
    if not b_qp or len(b_qp[0]) != len(wrench):
        raise ValueError(f"B_qp shape is {matrix_shape(b_qp)}, expected second dimension {len(wrench)}")

    ax0 = matvec(a_qp, x0, "A_qp*x0")
    bw = matvec(b_qp, wrench, "B_qp*wrench")
    x_recon = [left + right for left, right in zip(ax0, bw)]
    stored = predicted_state_vector(log)
    if len(stored) != len(x_recon):
        raise ValueError(f"stored prediction size {len(stored)} does not match reconstructed {len(x_recon)}")

    steps = int(log.get("metadata", {}).get("horizon_steps", len(x_recon) // STATE_DIM))
    if len(x_recon) != steps * STATE_DIM:
        raise ValueError(f"reconstructed state size {len(x_recon)} does not match horizon {steps}")

    return x0, reshape_vector(x_recon, steps, STATE_DIM), reshape_vector(stored, steps, STATE_DIM), wrench


def matrix_shape(matrix) -> tuple[int, int]:
    if not matrix:
        return (0, 0)
    return (len(matrix), len(matrix[0]))


def matvec(matrix, vector, name: str) -> list[float]:
    if matrix and len(matrix[0]) != len(vector):
        raise ValueError(f"{name} dimension mismatch: {matrix_shape(matrix)} times {len(vector)}")
    return [sum(row[col] * vector[col] for col in range(len(vector))) for row in matrix]


def reshape_vector(vector: list[float], rows: int, cols: int) -> list[list[float]]:
    return [vector[row * cols : (row + 1) * cols] for row in range(rows)]


def flatten_matrix(matrix: list[list[float]]) -> list[float]:
    return [item for row in matrix for item in row]


def max_abs_matrix_delta(left: list[list[float]], right: list[list[float]]) -> float:
    return max(
        abs(left_row[col] - right_row[col])
        for left_row, right_row in zip(left, right)
        for col in range(len(left_row))
    )


def plot_trajectory(log_path: Path, log: dict, x0, x_horizon, max_abs_error: float, save_path: Path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for plotting") from exc

    dt = float(log.get("metadata", {}).get("dt_mpc", 1.0))
    robot_type = (
        log.get("metadata", {}).get("robot_type")
        or log.get("controller_config", {}).get("robot_type")
        or "unknown robot"
    )
    states = [x0] + x_horizon
    time = [step * dt for step in range(len(states))]

    fig = plt.figure(figsize=(11, 8))
    fig.suptitle(
        f"{robot_type} | {log_path.name} | max reconstruction error {max_abs_error:.3e}"
    )

    ax_xy = fig.add_subplot(2, 2, 1)
    ax_xy.plot([state[3] for state in states], [state[4] for state in states], marker="o")
    ax_xy.set_xlabel("COM x_W [m]")
    ax_xy.set_ylabel("COM y_W [m]")
    ax_xy.set_title("Reduced-body COM XY")
    ax_xy.axis("equal")
    ax_xy.grid(True)

    ax_z = fig.add_subplot(2, 2, 2)
    ax_z.plot(time, [state[5] for state in states], marker="o")
    ax_z.set_xlabel("time [s]")
    ax_z.set_ylabel("COM z_W [m]")
    ax_z.set_title("COM height")
    ax_z.grid(True)

    ax_rpy = fig.add_subplot(2, 2, 3)
    ax_rpy.plot(time, [state[0] for state in states], marker="o", label="roll")
    ax_rpy.plot(time, [state[1] for state in states], marker="o", label="pitch")
    ax_rpy.plot(time, [state[2] for state in states], marker="o", label="yaw")
    ax_rpy.set_xlabel("time [s]")
    ax_rpy.set_ylabel("angle [rad]")
    ax_rpy.set_title("SRB attitude")
    ax_rpy.legend()
    ax_rpy.grid(True)

    ax_vel = fig.add_subplot(2, 2, 4)
    ax_vel.plot(time, [state[9] for state in states], marker="o", label="vx")
    ax_vel.plot(time, [state[10] for state in states], marker="o", label="vy")
    ax_vel.plot(time, [state[11] for state in states], marker="o", label="vz")
    ax_vel.set_xlabel("time [s]")
    ax_vel.set_ylabel("linear velocity [m/s]")
    ax_vel.set_title("COM velocity")
    ax_vel.legend()
    ax_vel.grid(True)

    fig.tight_layout()

    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=160)
    print(f"saved plot: {save_path}")
    plt.close(fig)


def main() -> int:
    if len(sys.argv) > 2:
        print("Usage: python test/standing_debug/srb_reconstruct.py [standing_mpc_debug.json]", file=sys.stderr)
        return 1

    log_path = Path(sys.argv[1]) if len(sys.argv) == 2 else latest_log_path()
    log_path = log_path.expanduser().resolve()
    log = load_json(log_path)
    x0, x_horizon, stored, wrench = reconstruct(log)
    max_abs_error = max_abs_matrix_delta(x_horizon, stored)
    plot_path = default_plot_path()

    print(f"log: {log_path}")
    print(f"horizon steps: {len(x_horizon)}")
    print(f"wrench dimension: {len(wrench)}")
    print(f"max |reconstructed - logged|: {max_abs_error:.12e}")

    if max_abs_error > 1e-9:
        print("ERROR: reconstruction error exceeds tolerance 1.0e-9", file=sys.stderr)
        return 2

    plot_trajectory(log_path, log, x0, x_horizon, max_abs_error, plot_path)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"srb_reconstruct.py: {exc}", file=sys.stderr)
        raise SystemExit(1)
