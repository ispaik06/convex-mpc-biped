from __future__ import annotations

import argparse
from pathlib import Path
import time

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot gait swing/hold trace CSV")
    parser.add_argument(
        "--csv",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "build" / "gait_swing_hold_trace.csv",
        help="Path to the trace CSV written by main_gait_swing_hold_test",
    )
    parser.add_argument(
        "--watch",
        action="store_true",
        help="Continuously refresh the plot while the simulator is running",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.15,
        help="Refresh interval in seconds when --watch is enabled",
    )
    parser.add_argument("--figwidth", type=float, default=9.0, help="Figure width in inches")
    parser.add_argument("--figheight", type=float, default=6.5, help="Figure height in inches")
    parser.add_argument(
        "--auto-scale",
        action="store_true",
        help="Keep axes auto-scaling on every redraw (default: lock scale after first data)",
    )
    return parser.parse_args()


def read_trace(csv_path: Path):
    if not csv_path.exists():
        return []

    lines = csv_path.read_text().strip().splitlines()
    if len(lines) <= 1:
        return []

    samples = []
    for row in lines[1:]:
        parts = row.split(",")
        if len(parts) == 9:
            samples.append(
                {
                    "segment": int(parts[0]),
                    "time": float(parts[1]),
                    "phase": parts[2],
                    "desired": (float(parts[3]), float(parts[4]), float(parts[5])),
                    "actual": (float(parts[6]), float(parts[7]), float(parts[8])),
                }
            )
            continue

        # Backward compatibility with previous CSV format:
        # time,phase,desired_x,desired_y,desired_z,actual_x,actual_y,actual_z
        if len(parts) == 8:
            samples.append(
                {
                    "segment": 1,
                    "time": float(parts[0]),
                    "phase": parts[1],
                    "desired": (float(parts[2]), float(parts[3]), float(parts[4])),
                    "actual": (float(parts[5]), float(parts[6]), float(parts[7])),
                }
            )
            continue
    return samples


def apply_fixed_limits(ax3d, axs, fixed_limits):
    if fixed_limits is None:
        return

    ax3d.set_xlim(fixed_limits["3d"]["x"])
    ax3d.set_ylim(fixed_limits["3d"]["y"])
    ax3d.set_zlim(fixed_limits["3d"]["z"])

    for idx, key in enumerate(["x", "y", "z"]):
        axs[idx].set_xlim(fixed_limits["2d"][key]["x"])
        axs[idx].set_ylim(fixed_limits["2d"][key]["y"])


def snapshot_limits(ax3d, axs):
    return {
        "3d": {
            "x": ax3d.get_xlim(),
            "y": ax3d.get_ylim(),
            "z": ax3d.get_zlim(),
        },
        "2d": {
            "x": {"x": axs[0].get_xlim(), "y": axs[0].get_ylim()},
            "y": {"x": axs[1].get_xlim(), "y": axs[1].get_ylim()},
            "z": {"x": axs[2].get_xlim(), "y": axs[2].get_ylim()},
        },
    }


def update_plot(ax3d, axs, samples, last_segment, fixed_limits, auto_scale):
    ax3d.cla()
    for ax in axs:
        ax.cla()

    ax3d.set_title("Left swing trace")
    ax3d.set_xlabel("x [m]")
    ax3d.set_ylabel("y [m]")
    ax3d.set_zlabel("z [m]")
    ax3d.grid(True)

    if not samples:
        ax3d.text2D(0.1, 0.9, "No swing samples yet", transform=ax3d.transAxes)
        for ax, label in zip(axs, ["x", "y", "z"]):
            ax.set_title(f"{label}(t)")
            ax.set_xlabel("time [s]")
            ax.set_ylabel(f"{label} [m]")
            ax.grid(True)
        apply_fixed_limits(ax3d, axs, fixed_limits)
        return last_segment, fixed_limits

    latest_segment = samples[-1]["segment"]
    latest_phase = samples[-1]["phase"]
    if last_segment is None or latest_segment != last_segment or latest_phase == "hold":
        last_segment = latest_segment

    active_samples = [sample for sample in samples if sample["segment"] == last_segment and sample["phase"] == "swing"]
    if not active_samples:
        ax3d.text2D(0.1, 0.9, "Hold phase", transform=ax3d.transAxes)
        for ax, label in zip(axs, ["x", "y", "z"]):
            ax.set_title(f"{label}(t)")
            ax.set_xlabel("time [s]")
            ax.set_ylabel(f"{label} [m]")
            ax.grid(True)
        apply_fixed_limits(ax3d, axs, fixed_limits)
        return last_segment, fixed_limits

    desired = list(zip(*[s["desired"] for s in active_samples]))
    actual = list(zip(*[s["actual"] for s in active_samples]))
    times = [s["time"] for s in active_samples]

    ax3d.plot(actual[0], actual[1], actual[2], "b-", label="actual foot")
    ax3d.plot(desired[0], desired[1], desired[2], "r--", label="desired swing")
    ax3d.scatter(actual[0][0], actual[1][0], actual[2][0], c="b", marker="o")
    ax3d.scatter(desired[0][-1], desired[1][-1], desired[2][-1], c="r", marker="s")
    ax3d.legend(loc="best")

    axis_labels = ["x", "y", "z"]
    colors = ["tab:blue", "tab:orange", "tab:green"]
    for idx, ax in enumerate(axs):
        ax.set_title(f"{axis_labels[idx]}(t)")
        ax.set_xlabel("time [s]")
        ax.set_ylabel(f"{axis_labels[idx]} [m]")
        ax.grid(True)
        ax.plot(times, actual[idx], color=colors[idx], label=f"actual {axis_labels[idx]}")
        ax.plot(times, desired[idx], color="tab:red", linestyle="--", label=f"desired {axis_labels[idx]}")
        ax.legend(loc="best")

    if fixed_limits is None and not auto_scale:
        fixed_limits = snapshot_limits(ax3d, axs)

    apply_fixed_limits(ax3d, axs, fixed_limits)
    return last_segment, fixed_limits


def main() -> int:
    args = parse_args()
    if args.watch:
        plt.ion()

    fig = plt.figure(figsize=(args.figwidth, args.figheight), constrained_layout=True)
    try:
        fig.canvas.manager.resize(int(args.figwidth * 100), int(args.figheight * 100))
    except Exception:
        pass
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax_x = fig.add_subplot(2, 2, 2)
    ax_y = fig.add_subplot(2, 2, 3)
    ax_z = fig.add_subplot(2, 2, 4)
    plt.show(block=False)

    last_segment = None
    fixed_limits = None
    last_mtime_ns = -1
    try:
        while True:
            mtime_ns = args.csv.stat().st_mtime_ns if args.csv.exists() else -1
            should_redraw = (not args.watch) or (mtime_ns != last_mtime_ns)
            if should_redraw:
                samples = read_trace(args.csv)
                last_segment, fixed_limits = update_plot(
                    ax3d,
                    [ax_x, ax_y, ax_z],
                    samples,
                    last_segment,
                    fixed_limits,
                    args.auto_scale,
                )
                plt.pause(0.001)
                last_mtime_ns = mtime_ns

            if not args.watch:
                break
            time.sleep(args.interval)
    except KeyboardInterrupt:
        pass

    plt.ioff()
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
