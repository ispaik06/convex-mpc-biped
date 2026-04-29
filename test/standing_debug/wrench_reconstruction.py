"""Reconstruct QP wrench from Jacobian-transpose torque projection."""

from __future__ import annotations

import csv
import json
import math
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEBUG_ROOT = PROJECT_ROOT / "logs" / "debug"
MAPPING_HELPER = PROJECT_ROOT / "build" / "test" / "standing_debug" / "wrench_mapping_probe"
LOG_DIRS = (DEBUG_ROOT / "standing_mpc", DEBUG_ROOT / "walking_mpc")
LOG_PATTERNS = ("standing_mpc_debug_*.json", "walking_mpc_debug_*.json")

INPUT_COMPONENTS = (
    ("L_Fx", "N"),
    ("L_Fy", "N"),
    ("L_Fz", "N"),
    ("R_Fx", "N"),
    ("R_Fy", "N"),
    ("R_Fz", "N"),
    ("L_Mx", "N m"),
    ("L_My", "N m"),
    ("L_Mz", "N m"),
    ("R_Mx", "N m"),
    ("R_My", "N m"),
    ("R_Mz", "N m"),
)


@dataclass(frozen=True)
class OutputPaths:
    report: Path
    csv: Path
    plot: Path


@dataclass
class ReconstructionResult:
    qp_wrench: object
    tau_from_qp: object
    reconstructed_wrench: object
    wrench_error: object
    singular_values: object
    rank: int
    condition_number: float
    max_abs_wrench_error: float
    rms_wrench_error: float
    max_abs_tau_roundtrip_error: float
    logged_tau_delta: float | None


def timestamp_token() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def latest_log_path() -> Path:
    paths = [
        path
        for log_dir in LOG_DIRS
        for pattern in LOG_PATTERNS
        for path in log_dir.glob(pattern)
    ]
    if not paths:
        raise FileNotFoundError(f"no MPC debug logs found under {DEBUG_ROOT}")
    return max(paths, key=lambda path: path.stat().st_mtime)


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def normalized_locomotion_mode(log: dict, log_path: Path) -> str:
    mode = (
        log.get("metadata", {}).get("locomotion_mode")
        or log.get("controller_config", {}).get("locomotion_mode")
        or ""
    )
    if mode in {"walking", "walk"}:
        return "walking"
    if mode in {"standing", "stand"}:
        return "standing"
    return "walking" if "walking_mpc" in log_path.parts else "standing"


def short_locomotion_prefix(mode: str) -> str:
    return "walk" if mode == "walking" else "stand"


def default_output_paths(log_path: Path, log: dict) -> OutputPaths:
    mode = normalized_locomotion_mode(log, log_path)
    debug_dir = "walking_mpc" if mode == "walking" else "standing_mpc"
    prefix = f"{short_locomotion_prefix(mode)}_wrench_reconstruction_{timestamp_token()}"
    root = DEBUG_ROOT / debug_dir / "wrench_reconstruction"
    report_dir = root / "reports"
    csv_dir = root / "csv"
    plot_dir = root / "plots"
    for directory in (report_dir, csv_dir, plot_dir):
        directory.mkdir(parents=True, exist_ok=True)
    return OutputPaths(
        report=report_dir / f"{prefix}.md",
        csv=csv_dir / f"{prefix}.csv",
        plot=plot_dir / f"{prefix}.png",
    )


def matrix_like(value, name: str):
    if isinstance(value, dict) and {"rows", "cols", "data"}.issubset(value):
        rows = int(value["rows"])
        cols = int(value["cols"])
        data = value["data"]
        if data and all(isinstance(row, list) for row in data):
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


def wrench_to_torque_section(log: dict) -> dict:
    section = log.get("wrench_to_torque")
    if isinstance(section, dict):
        return section
    legacy_section = log.get("standing_wrench_to_torque")
    if isinstance(legacy_section, dict):
        return legacy_section
    return {}


def helper_mapping_section(log_path: Path, legacy_section: dict) -> dict:
    if not MAPPING_HELPER.exists():
        raise ValueError(
            "wrench_to_tau_jacobian is not available in the log and "
            f"mapping helper is missing: {MAPPING_HELPER}"
        )
    if not MAPPING_HELPER.is_file():
        raise ValueError(f"mapping helper path is not a file: {MAPPING_HELPER}")

    completed = subprocess.run(
        [str(MAPPING_HELPER), str(log_path)],
        check=False,
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        message = completed.stderr.strip() or completed.stdout.strip() or "unknown helper failure"
        raise ValueError(f"failed to rebuild wrench_to_tau_jacobian from MuJoCo: {message}")

    try:
        section = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise ValueError("mapping helper returned invalid JSON") from exc

    if isinstance(legacy_section, dict) and "actual_leg_tau_vector" in legacy_section:
        section.setdefault("actual_leg_tau_vector", legacy_section["actual_leg_tau_vector"])
    return section


def format_float(value: float | None) -> str:
    if value is None:
        return "n/a"
    if math.isinf(value):
        return "inf"
    if math.isnan(value):
        return "nan"
    return f"{value:.9e}"


def component_rows(result: ReconstructionResult) -> list[dict[str, str | float]]:
    rows = []
    for index, (name, unit) in enumerate(INPUT_COMPONENTS):
        error = float(result.wrench_error[index])
        rows.append(
            {
                "component": name,
                "unit": unit,
                "qp_wrench": float(result.qp_wrench[index]),
                "reconstructed_wrench": float(result.reconstructed_wrench[index]),
                "error": error,
                "abs_error": abs(error),
            }
        )
    return rows


def reconstruct(log_path: Path, log: dict) -> tuple[ReconstructionResult, dict]:
    try:
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("numpy is required for wrench reconstruction") from exc

    section = wrench_to_torque_section(log)
    if "wrench_to_tau_jacobian" not in section:
        section = helper_mapping_section(log_path, section)
    if "wrench_to_tau_jacobian" not in section:
        reason = section.get("unavailable_reason") or section.get("mapping") or "mapping is missing"
        raise ValueError(f"wrench_to_tau_jacobian is not available: {reason}")

    mapping = np.asarray(
        matrix_like(section["wrench_to_tau_jacobian"], "wrench_to_tau_jacobian"),
        dtype=float,
    )
    qp_wrench = np.asarray(vector_like(log["solution"]["first_wrench"], "solution.first_wrench"),
                           dtype=float)
    if mapping.ndim != 2 or mapping.shape[1] != len(INPUT_COMPONENTS):
        raise ValueError(
            f"wrench_to_tau_jacobian shape is {mapping.shape}, expected N x {len(INPUT_COMPONENTS)}"
        )
    if qp_wrench.shape != (len(INPUT_COMPONENTS),):
        raise ValueError(f"solution.first_wrench has shape {qp_wrench.shape}, expected 12")

    tau_from_qp = mapping @ qp_wrench
    pinv = np.linalg.pinv(mapping, rcond=1.0e-9)
    reconstructed_wrench = pinv @ tau_from_qp
    tau_roundtrip = mapping @ reconstructed_wrench
    wrench_error = reconstructed_wrench - qp_wrench
    singular_values = np.linalg.svd(mapping, compute_uv=False)
    rank = int(np.linalg.matrix_rank(mapping, tol=1.0e-9))
    nonzero_singular_values = [value for value in singular_values if value > 1.0e-12]
    if nonzero_singular_values:
        condition_number = float(max(nonzero_singular_values) / min(nonzero_singular_values))
    else:
        condition_number = math.inf

    logged_tau_delta = None
    actual_leg_tau = section.get("actual_leg_tau_vector")
    if actual_leg_tau is not None:
        logged_tau = np.asarray(vector_like(actual_leg_tau, "actual_leg_tau_vector"), dtype=float)
        if logged_tau.shape == tau_from_qp.shape:
            logged_tau_delta = float(np.max(np.abs(tau_from_qp - logged_tau)))

    result = ReconstructionResult(
        qp_wrench=qp_wrench,
        tau_from_qp=tau_from_qp,
        reconstructed_wrench=reconstructed_wrench,
        wrench_error=wrench_error,
        singular_values=singular_values,
        rank=rank,
        condition_number=condition_number,
        max_abs_wrench_error=float(np.max(np.abs(wrench_error))),
        rms_wrench_error=float(np.sqrt(np.mean(wrench_error * wrench_error))),
        max_abs_tau_roundtrip_error=float(np.max(np.abs(tau_roundtrip - tau_from_qp))),
        logged_tau_delta=logged_tau_delta,
    )
    return result, section


def write_csv(path: Path, log_path: Path, log: dict, section: dict, result: ReconstructionResult) -> None:
    mode = normalized_locomotion_mode(log, log_path)
    with path.open("w", encoding="utf-8", newline="") as handle:
        handle.write(f"# source_json_file={log_path.name}\n")
        handle.write(f"# source_json_path={log_path.resolve()}\n")
        handle.write(f"# robot_type={log.get('metadata', {}).get('robot_type', 'unknown')}\n")
        handle.write(f"# locomotion_mode={mode}\n")
        handle.write(f"# mapping_source={section.get('source', 'unknown')}\n")
        handle.write(f"# mapping_shape={len(result.tau_from_qp)}x{len(INPUT_COMPONENTS)}\n")
        writer = csv.DictWriter(
            handle,
            fieldnames=(
                "component",
                "unit",
                "qp_wrench",
                "reconstructed_wrench",
                "error",
                "abs_error",
            ),
        )
        writer.writeheader()
        writer.writerows(component_rows(result))


def write_markdown_report(
    path: Path,
    log_path: Path,
    log: dict,
    section: dict,
    result: ReconstructionResult | None,
    outputs: OutputPaths,
    unavailable_reason: str | None = None,
) -> None:
    metadata = log.get("metadata", {})
    mode = normalized_locomotion_mode(log, log_path)
    robot_type = metadata.get("robot_type", "unknown")
    with path.open("w", encoding="utf-8") as out:
        out.write("# Wrench Reconstruction Report\n\n")
        out.write("## Source\n\n")
        out.write("| Field | Value |\n")
        out.write("| --- | --- |\n")
        out.write(f"| Source log | `{log_path.resolve()}` |\n")
        out.write(f"| Robot type | `{robot_type}` |\n")
        out.write(f"| Locomotion mode | `{mode}` |\n")
        out.write(f"| Mapping source | `{section.get('source', 'unknown')}` |\n")
        out.write(f"| Input order | `{section.get('input_order', '[F_left, F_right, M_left, M_right]')}` |\n\n")

        out.write("## Method\n\n")
        out.write("The analysis uses the logged Jacobian-transpose map `A` from MPC wrench to leg torque:\n\n")
        out.write("$$\n")
        out.write("tau_{qp} = A w_{qp}, \\qquad \\hat{w} = A^+ tau_{qp} = A^+ A w_{qp}\n")
        out.write("$$\n\n")
        out.write("The plotted error is:\n\n")
        out.write("$$\n")
        out.write("e_w = \\hat{w} - w_{qp}\n")
        out.write("$$\n\n")
        out.write("For two 5-DoF legs, `A` is usually `10 x 12`, so `A^+ A` is a projection into the wrench subspace visible through joint torque.\n\n")

        if result is None:
            out.write("## Status\n\n")
            out.write("`wrench_to_tau_jacobian` was not available in this log, so the reconstruction plot was skipped.\n\n")
            if unavailable_reason:
                out.write(f"Reason: `{unavailable_reason}`\n\n")
            return

        out.write("## Summary\n\n")
        out.write("| Metric | Value |\n")
        out.write("| --- | ---: |\n")
        out.write(f"| Torque dimension | {len(result.tau_from_qp)} |\n")
        out.write(f"| Wrench dimension | {len(INPUT_COMPONENTS)} |\n")
        out.write(f"| Rank(A) | {result.rank} |\n")
        out.write(f"| cond(A) over nonzero singular values | {format_float(result.condition_number)} |\n")
        out.write(f"| max abs wrench error | {format_float(result.max_abs_wrench_error)} |\n")
        out.write(f"| RMS wrench error | {format_float(result.rms_wrench_error)} |\n")
        out.write(f"| max abs torque roundtrip error | {format_float(result.max_abs_tau_roundtrip_error)} |\n")
        out.write(f"| max abs tau_qp - logged actual leg tau | {format_float(result.logged_tau_delta)} |\n\n")

        out.write("## Wrench Components\n\n")
        out.write("| Component | Unit | QP wrench | reconstructed | error |\n")
        out.write("| --- | --- | ---: | ---: | ---: |\n")
        for row in component_rows(result):
            out.write(
                f"| `{row['component']}` | {row['unit']} | "
                f"{format_float(float(row['qp_wrench']))} | "
                f"{format_float(float(row['reconstructed_wrench']))} | "
                f"{format_float(float(row['error']))} |\n"
            )
        out.write("\n")

        out.write("## Torque From QP Wrench\n\n")
        out.write("| Index | tau_qp |\n")
        out.write("| ---: | ---: |\n")
        for index, value in enumerate(result.tau_from_qp):
            out.write(f"| {index} | {format_float(float(value))} |\n")
        out.write("\n")

        out.write("## Singular Values\n\n")
        out.write("| Index | sigma |\n")
        out.write("| ---: | ---: |\n")
        for index, value in enumerate(result.singular_values):
            out.write(f"| {index} | {format_float(float(value))} |\n")
        out.write("\n")

        out.write("## Outputs\n\n")
        out.write("| Artifact | Path |\n")
        out.write("| --- | --- |\n")
        out.write(f"| CSV | `{outputs.csv.resolve()}` |\n")
        out.write(f"| Plot | `{outputs.plot.resolve()}` |\n")


def plot_result(path: Path, log_path: Path, log: dict, result: ReconstructionResult) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for plotting") from exc

    labels = [name for name, _ in INPUT_COMPONENTS]
    x_values = list(range(len(labels)))
    width = 0.38
    errors = [float(value) for value in result.wrench_error]
    robot_type = log.get("metadata", {}).get("robot_type", "unknown robot")
    mode = normalized_locomotion_mode(log, log_path)

    fig, axes = plt.subplots(2, 1, figsize=(13, 8.5), sharex=True)
    fig.suptitle(
        f"{robot_type} | {mode} | {log_path.name}\n"
        f"max abs wrench error {result.max_abs_wrench_error:.3e}",
        fontsize=12,
    )

    axes[0].bar(
        [value - width / 2 for value in x_values],
        [float(value) for value in result.qp_wrench],
        width,
        label="QP wrench",
        color="#4c78a8",
    )
    axes[0].bar(
        [value + width / 2 for value in x_values],
        [float(value) for value in result.reconstructed_wrench],
        width,
        label="pinv(J^T) tau",
        color="#f58518",
    )
    axes[0].axhline(0.0, color="#222222", linewidth=0.8)
    axes[0].set_ylabel("N / N m")
    axes[0].set_title("QP wrench vs reconstructed wrench")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[0].legend()

    axes[1].bar(x_values, errors, color="#e45756")
    axes[1].axhline(0.0, color="#222222", linewidth=0.8)
    axes[1].set_xticks(x_values)
    axes[1].set_xticklabels(labels, rotation=35, ha="right")
    axes[1].set_ylabel("N / N m")
    axes[1].set_title("Reconstruction error")
    axes[1].grid(True, axis="y", alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.92])
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    plt.close(fig)


def run(log_path: Path) -> None:
    log = load_json(log_path)
    outputs = default_output_paths(log_path, log)
    section = wrench_to_torque_section(log)
    try:
        result, section = reconstruct(log_path, log)
    except ValueError as exc:
        write_markdown_report(outputs.report, log_path, log, section, None, outputs, str(exc))
        print(f"wrench reconstruction report: {outputs.report.resolve()}")
        print(f"wrench reconstruction: skipped plot ({exc})")
        return

    write_csv(outputs.csv, log_path, log, section, result)
    plot_result(outputs.plot, log_path, log, result)
    write_markdown_report(outputs.report, log_path, log, section, result, outputs)
    print(f"wrench reconstruction report: {outputs.report.resolve()}")
    print(f"wrench reconstruction csv: {outputs.csv.resolve()}")
    print(f"wrench reconstruction plot: {outputs.plot.resolve()}")


def main() -> int:
    if len(sys.argv) == 2 and sys.argv[1] in {"-h", "--help"}:
        print("Usage: python test/standing_debug/wrench_reconstruction.py [mpc_debug.json]")
        return 0
    if len(sys.argv) > 2:
        print("Usage: python test/standing_debug/wrench_reconstruction.py [mpc_debug.json]", file=sys.stderr)
        return 1

    log_path = Path(sys.argv[1]) if len(sys.argv) == 2 else latest_log_path()
    run(log_path.expanduser().resolve())
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"wrench_reconstruction.py: {exc}", file=sys.stderr)
        raise SystemExit(1)
