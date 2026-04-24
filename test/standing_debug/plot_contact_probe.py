"""Plot desired vs MuJoCo-realized contact wrench from stand_contact_probe CSV."""

from __future__ import annotations

import csv
import sys
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CSV_DIR = PROJECT_ROOT / "logs" / "debug" / "standing_mpc" / "contact_probe" / "csv"
DEFAULT_PLOT_DIR = PROJECT_ROOT / "logs" / "debug" / "standing_mpc" / "contact_probe" / "plots"


def timestamp_token() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def latest_csv_path() -> Path:
    paths = sorted(DEFAULT_CSV_DIR.glob("stand_contact_probe_*.csv"))
    if not paths:
        raise FileNotFoundError(f"no contact probe CSV files found in {DEFAULT_CSV_DIR}")
    return paths[-1]


def default_plot_path(csv_path: Path) -> Path:
    token = csv_path.stem.removeprefix("stand_contact_probe_")
    if token == csv_path.stem:
        token = timestamp_token()
    return DEFAULT_PLOT_DIR / f"stand_contact_probe_{token}.png"


def parse_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    return float(value)


def read_rows(csv_path: Path) -> tuple[list[dict], dict[str, str]]:
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

    rows = list(csv.DictReader(lines))

    for row in rows:
        row["desired"] = parse_float(row.get("desired"))
        row["measured"] = parse_float(row.get("measured"))
        row["error"] = parse_float(row.get("error"))
        row["contacts"] = int(row["contacts"])
        row["comparable"] = row["comparable"] == "1"
    return rows, metadata


def short_label(row: dict) -> str:
    side = "L" if row["side"] == "left" else "R"
    prefix = "F" if row["quantity"] == "force" else "M"
    return f"{side} {prefix}{row['axis']}"


def select_rows(rows: list[dict], quantity: str, measurement_point: str | None = None, comparable=False):
    selected = [row for row in rows if row["quantity"] == quantity]
    if measurement_point is not None:
        selected = [row for row in selected if row["measurement_point"] == measurement_point]
    if comparable:
        selected = [row for row in selected if row["comparable"]]
    return selected


def plot_pair(ax, rows: list[dict], title: str, unit: str):
    labels = [short_label(row) for row in rows]
    desired = [row["desired"] for row in rows]
    measured = [row["measured"] for row in rows]
    x = list(range(len(rows)))
    width = 0.38

    ax.bar([value - width / 2 for value in x], desired, width, label="QP desired", color="#4c78a8")
    ax.bar([value + width / 2 for value in x], measured, width, label="MuJoCo realized", color="#f58518")
    ax.axhline(0.0, color="#222222", linewidth=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.set_ylabel(unit)
    ax.set_title(title)
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()


def plot_error(ax, rows: list[dict], title: str, unit: str):
    labels = [short_label(row) for row in rows]
    errors = [row["error"] for row in rows]
    colors = ["#54a24b" if value is not None and abs(value) < 1e-9 else "#e45756" for value in errors]
    x = list(range(len(rows)))

    ax.bar(x, errors, color=colors)
    ax.axhline(0.0, color="#222222", linewidth=0.8)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.set_ylabel(unit)
    ax.set_title(title)
    ax.grid(True, axis="y", alpha=0.3)


def plot_contact_counts(ax, rows: list[dict]):
    seen = {}
    for row in rows:
        key = (row["side"], row["measurement_point"])
        seen[key] = row["contacts"]

    labels = [("L" if side == "left" else "R") + "\n" + point for side, point in seen]
    values = list(seen.values())
    ax.bar(range(len(values)), values, color="#72b7b2")
    ax.set_xticks(range(len(values)))
    ax.set_xticklabels(labels)
    ax.set_ylabel("contacts")
    ax.set_title("Contact count")
    ax.grid(True, axis="y", alpha=0.3)


def plot_moment_reference_delta(ax, rows: list[dict]):
    site_rows = select_rows(rows, "moment", "foot_site")
    com_rows = select_rows(rows, "moment", "foot_link_com")
    site_by_key = {(row["side"], row["axis"]): row for row in site_rows}
    pairs = [(row, site_by_key[(row["side"], row["axis"])]) for row in com_rows]
    labels = [short_label(com_row) for com_row, _ in pairs]
    deltas = [com_row["measured"] - site_row["measured"] for com_row, site_row in pairs]

    ax.bar(range(len(deltas)), deltas, color="#b279a2")
    ax.axhline(0.0, color="#222222", linewidth=0.8)
    ax.set_xticks(range(len(deltas)))
    ax.set_xticklabels(labels, rotation=35, ha="right")
    ax.set_ylabel("N m")
    ax.set_title("Moment: foot_link_com - foot_site")
    ax.grid(True, axis="y", alpha=0.3)


def plot(csv_path: Path, save_path: Path) -> None:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError("matplotlib is required for plotting") from exc

    rows, metadata = read_rows(csv_path)
    force_rows = select_rows(rows, "force", "foot_site", comparable=True)
    moment_rows = select_rows(rows, "moment", comparable=True)
    if not force_rows:
        raise ValueError("CSV has no comparable force rows")
    if not moment_rows:
        raise ValueError("CSV has no comparable moment rows")

    moment_reference = moment_rows[0]["measurement_point"]
    source_json = metadata.get("source_json_file", "unknown json")
    robot_type = metadata.get("robot_type", "unknown robot")
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))
    fig.suptitle(f"{csv_path.name}\n{robot_type}\nsource json: {source_json}", fontsize=12)

    plot_pair(axes[0, 0], force_rows, "Force at foot_site", "N")
    plot_error(axes[1, 0], force_rows, "Force error at foot_site", "N")
    plot_pair(axes[0, 1], moment_rows, f"Moment about {moment_reference}", "N m")
    plot_error(axes[1, 1], moment_rows, f"Moment error about {moment_reference}", "N m")
    plot_contact_counts(axes[0, 2], rows)
    plot_moment_reference_delta(axes[1, 2], rows)

    fig.tight_layout(rect=[0, 0, 1, 0.92])
    save_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(save_path, dpi=160)
    plt.close(fig)
    print(f"saved plot: {save_path}")


def main() -> int:
    if len(sys.argv) > 3:
        print("Usage: python test/standing_debug/plot_contact_probe.py [contact_probe.csv] [output.png]", file=sys.stderr)
        return 1

    csv_path = Path(sys.argv[1]) if len(sys.argv) >= 2 else latest_csv_path()
    csv_path = csv_path.expanduser().resolve()
    save_path = Path(sys.argv[2]).expanduser().resolve() if len(sys.argv) == 3 else default_plot_path(csv_path)
    plot(csv_path, save_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
