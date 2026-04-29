"""Plot SRB-only receding-horizon probe CSV output."""

from __future__ import annotations

import csv
import sys
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEBUG_ROOT = PROJECT_ROOT / "logs" / "debug"
CSV_DIRS = (
    DEBUG_ROOT / "standing_mpc" / "receding_horizon" / "csv",
    DEBUG_ROOT / "walking_mpc" / "receding_horizon" / "csv",
)
CSV_PATTERNS = ("stand_rh_*.csv", "walk_rh_*.csv")

STATE_GRID = [
    ("roll", "pitch", "yaw"),
    ("px", "py", "pz"),
    ("omega_x", "omega_y", "omega_z"),
    ("vx", "vy", "vz"),
]
STATE_ROW_UNITS = ("rad", "m", "rad/s", "m/s")

WRENCH_GRID = [
    ("L_Fx", "L_Fy", "L_Fz"),
    ("R_Fx", "R_Fy", "R_Fz"),
    ("L_Mx", "L_My", "L_Mz"),
    ("R_Mx", "R_My", "R_Mz"),
]


def timestamp_token() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def latest_csv_path() -> Path:
    paths = [
        path
        for csv_dir in CSV_DIRS
        for pattern in CSV_PATTERNS
        for path in csv_dir.glob(pattern)
    ]
    if not paths:
        raise FileNotFoundError(f"no receding-horizon CSV files found under {DEBUG_ROOT}")
    return max(paths, key=lambda path: path.stat().st_mtime)


def default_plot_paths(csv_path: Path) -> tuple[Path, Path, Path]:
    token = csv_path.stem.removeprefix("stand_rh_")
    prefix = "stand_rh_"
    if token == csv_path.stem:
        token = csv_path.stem.removeprefix("walk_rh_")
        prefix = "walk_rh_"
    if token == csv_path.stem:
        token = timestamp_token()
        prefix = "rh_"
    run_dir = csv_path.parent.parent / "plots" / f"{prefix}{token}"
    return (
        run_dir / "states.png",
        run_dir / "wrench.png",
        run_dir / "metrics.png",
    )


def read_csv(csv_path: Path) -> tuple[list[dict[str, float | int]], dict[str, str]]:
    metadata: dict[str, str] = {}
    lines: list[str] = []
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        for raw_line in handle:
            line = raw_line.rstrip("\n")
            if line.startswith("#"):
                entry = line[1:].strip()
                if "=" in entry:
                    key, value = entry.split("=", 1)
                    metadata[key.strip()] = value.strip()
                continue
            lines.append(raw_line)

    rows: list[dict[str, float | int]] = []
    for row in csv.DictReader(lines):
        parsed: dict[str, float | int] = {}
        for key, value in row.items():
            if key in {"step", "solve_ok"}:
                parsed[key] = int(value)
            else:
                parsed[key] = float(value)
        rows.append(parsed)
    if not rows:
        raise ValueError(f"{csv_path} has no data rows")
    return rows, metadata


def title_prefix(csv_path: Path, metadata: dict[str, str]) -> str:
    source_json = metadata.get("source_json_file", "unknown json")
    robot_type = metadata.get("robot_type", "unknown robot")
    locomotion_mode = metadata.get("locomotion_mode", "unknown mode")
    return f"{csv_path.name}\n{robot_type} | {locomotion_mode}\nsource json: {source_json}"


def state_trajectory(rows: list[dict[str, float | int]], name: str, dt: float):
    times = [float(row["time"]) for row in rows]
    values = [float(row[name]) for row in rows]
    refs = [float(row[f"ref_{name}"]) for row in rows]

    last = rows[-1]
    if int(last["solve_ok"]) == 1:
        times.append(float(last["time"]) + dt)
        values.append(float(last[f"next_{name}"]))
        refs.append(float(last[f"ref_{name}"]))
    return times, values, refs


def plot_states(csv_path: Path, rows, metadata, save_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    dt = float(metadata.get("config_dt_mpc", 1.0))
    fig, axes = plt.subplots(4, 3, figsize=(15, 11), sharex=True)
    fig.suptitle(title_prefix(csv_path, metadata), fontsize=12)

    for row_index, names in enumerate(STATE_GRID):
        for col_index, name in enumerate(names):
            ax = axes[row_index, col_index]
            times, values, refs = state_trajectory(rows, name, dt)
            ax.plot(times, values, color="#4c78a8", linewidth=1.8, label="RH state")
            ax.plot(times, refs, color="#e45756", linestyle="--", linewidth=1.2, label="reference")
            ax.axhline(0.0, color="#222222", linewidth=0.6, alpha=0.35)
            ax.set_title(name)
            ax.grid(True, alpha=0.3)
            if row_index == len(STATE_GRID) - 1:
                ax.set_xlabel("rollout time [s]")
            if col_index == 0:
                ax.set_ylabel(STATE_ROW_UNITS[row_index])
            if row_index == 0 and col_index == 0:
                ax.legend(loc="best")

    fig.tight_layout(rect=[0, 0, 1, 0.93])
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=160)
    plt.close(fig)
    print(f"saved states plot: {save_path}")


def plot_wrench(csv_path: Path, rows, metadata, save_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [float(row["time"]) for row in rows if int(row["solve_ok"]) == 1]
    solved_rows = [row for row in rows if int(row["solve_ok"]) == 1]
    if not solved_rows:
        raise ValueError("no solved rows available for wrench plot")

    fig, axes = plt.subplots(4, 3, figsize=(15, 11), sharex=True)
    fig.suptitle(title_prefix(csv_path, metadata), fontsize=12)

    for row_index, names in enumerate(WRENCH_GRID):
        for col_index, name in enumerate(names):
            ax = axes[row_index, col_index]
            values = [float(row[name]) for row in solved_rows]
            ax.plot(times, values, color="#f58518", linewidth=1.8)
            ax.axhline(0.0, color="#222222", linewidth=0.7, alpha=0.45)
            ax.set_title(name)
            ax.grid(True, alpha=0.3)
            if row_index == len(WRENCH_GRID) - 1:
                ax.set_xlabel("rollout time [s]")
            if col_index == 0:
                ax.set_ylabel("N" if row_index < 2 else "N m")

    fig.tight_layout(rect=[0, 0, 1, 0.93])
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=160)
    plt.close(fig)
    print(f"saved wrench plot: {save_path}")


def plot_metrics(csv_path: Path, rows, metadata, save_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    times = [float(row["time"]) for row in rows]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    fig.suptitle(title_prefix(csv_path, metadata), fontsize=12)

    error_names = [
        "error_norm_rpy",
        "error_norm_position",
        "error_norm_omega",
        "error_norm_velocity",
        "error_norm_all",
    ]
    for name in error_names:
        axes[0, 0].plot(times, [float(row[name]) for row in rows], label=name)
    axes[0, 0].set_title("Current-state error norms")
    axes[0, 0].set_xlabel("rollout time [s]")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(fontsize=8)

    axes[0, 1].plot(times, [float(row["weighted_horizon_error_norm"]) for row in rows], color="#54a24b")
    axes[0, 1].set_title("Weighted horizon error norm")
    axes[0, 1].set_xlabel("rollout time [s]")
    axes[0, 1].grid(True, alpha=0.3)

    axes[1, 0].plot(times, [float(row["input_norm"]) for row in rows], color="#b279a2")
    axes[1, 0].set_title("Wrench horizon norm")
    axes[1, 0].set_xlabel("rollout time [s]")
    axes[1, 0].grid(True, alpha=0.3)

    axes[1, 1].plot(times, [float(row["weighted_input_norm"]) for row in rows], color="#9d755d")
    axes[1, 1].set_title("Weighted input norm")
    axes[1, 1].set_xlabel("rollout time [s]")
    axes[1, 1].grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.91])
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=160)
    plt.close(fig)
    print(f"saved metrics plot: {save_path}")


def plot(csv_path: Path, states_path: Path, wrench_path: Path, metrics_path: Path) -> None:
    rows, metadata = read_csv(csv_path)
    plot_states(csv_path, rows, metadata, states_path)
    plot_wrench(csv_path, rows, metadata, wrench_path)
    plot_metrics(csv_path, rows, metadata, metrics_path)


def main() -> int:
    if len(sys.argv) == 2 and sys.argv[1] in {"-h", "--help"}:
        print(
            "Usage: python test/standing_debug/plot_stand_rh.py "
            "[stand_rh.csv states.png wrench.png metrics.png]"
        )
        return 0

    if len(sys.argv) not in {1, 2, 5}:
        print(
            "Usage: python test/standing_debug/plot_stand_rh.py "
            "[stand_rh.csv states.png wrench.png metrics.png]",
            file=sys.stderr,
        )
        return 1

    csv_path = Path(sys.argv[1]) if len(sys.argv) >= 2 else latest_csv_path()
    csv_path = csv_path.expanduser().resolve()
    if len(sys.argv) == 5:
        states_path = Path(sys.argv[2]).expanduser().resolve()
        wrench_path = Path(sys.argv[3]).expanduser().resolve()
        metrics_path = Path(sys.argv[4]).expanduser().resolve()
    else:
        states_path, wrench_path, metrics_path = default_plot_paths(csv_path)

    plot(csv_path, states_path, wrench_path, metrics_path)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"plot_stand_rh.py: {exc}", file=sys.stderr)
        raise SystemExit(1)
