#!/usr/bin/env python3

import csv
import math
import statistics
import sys
from pathlib import Path


def as_float(row, key):
    value = row.get(key)
    if value in (None, ""):
        raise ValueError(key)
    return float(value)


def load_rows(path):
    rows = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            try:
                rows.append({key: as_float(raw, key) for key in reader.fieldnames})
            except (TypeError, ValueError):
                # The simulator may be interrupted while writing the last CSV line.
                continue
    return rows


def first_time(rows, predicate):
    for row in rows:
        if predicate(row):
            return row["time"]
    return None


def percentile(values, ratio):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(ratio * len(ordered))) - 1))
    return ordered[index]


def has_columns(rows, *keys):
    return bool(rows) and all(key in rows[0] for key in keys)


def side_swing_stats(rows, side):
    mode_key = f"{side}_mode"
    scheduled_key = f"{side}_cm_scheduled"
    foot_z_key = f"{side}_foot_z"
    swing_rows = [row for row in rows if row[mode_key] == 2]
    stance_rows = [row for row in rows if row[mode_key] == 3]
    scheduled_swing_rows = (
        [row for row in rows if row[scheduled_key] == 0]
        if has_columns(rows, scheduled_key)
        else []
    )

    stats = {
        "mode_swing_fraction": len(swing_rows) / len(rows) if rows else float("nan"),
        "scheduled_swing_fraction": (
            len(scheduled_swing_rows) / len(rows) if rows and scheduled_swing_rows else float("nan")
        ),
        "stance_samples": len(stance_rows),
        "swing_samples": len(swing_rows),
        "max_swing_foot_z": max((row[foot_z_key] for row in swing_rows), default=float("nan")),
        "p95_swing_foot_z": percentile([row[foot_z_key] for row in swing_rows], 0.95),
        "min_stance_foot_z": min((row[foot_z_key] for row in stance_rows), default=float("nan")),
        "max_stance_foot_z": max((row[foot_z_key] for row in stance_rows), default=float("nan")),
    }
    return stats


def summarize(path):
    rows = load_rows(path)
    if not rows:
        raise SystemExit(f"No telemetry rows found in {path}")

    walking = [row for row in rows if row["time"] >= 3.5]
    stance_rows = [
        row for row in walking
        if row["left_mode"] == 3 or row["right_mode"] == 3
    ]
    zero_stance = [
        row for row in walking
        if row["left_mode"] != 3 and row["right_mode"] != 3
    ]
    force_errors = []
    for row in walking:
        if row["left_mode"] == 3:
            force_errors.append(row["left_force_error_norm"])
        if row["right_mode"] == 3:
            force_errors.append(row["right_force_error_norm"])
    moment_errors = []
    if has_columns(rows,
                   "total_actual_mx", "total_actual_my", "total_actual_mz",
                   "total_desired_mx", "total_desired_my", "total_desired_mz"):
        for row in walking:
            moment_errors.append(math.sqrt(
                (row["total_actual_mx"] - row["total_desired_mx"]) ** 2 +
                (row["total_actual_my"] - row["total_desired_my"]) ** 2 +
                (row["total_actual_mz"] - row["total_desired_mz"]) ** 2
            ))

    max_abs_roll = max(abs(row["roll"]) for row in walking) if walking else float("nan")
    max_abs_pitch = max(abs(row["pitch"]) for row in walking) if walking else float("nan")
    min_com_z = min(row["com_z"] for row in walking) if walking else float("nan")
    max_xy_drift = max(
        math.hypot(row["com_x"] - walking[0]["com_x"], row["com_y"] - walking[0]["com_y"])
        for row in walking
    ) if walking else float("nan")

    print(f"Telemetry: {path}")
    print(f"samples={len(rows)} time=[{rows[0]['time']:.3f}, {rows[-1]['time']:.3f}]")
    print(
        "fall_flags "
        f"com_z<0.55={first_time(walking, lambda r: r['com_z'] < 0.55)} "
        f"com_z<0.35={first_time(walking, lambda r: r['com_z'] < 0.35)} "
        f"angle>0.6={first_time(walking, lambda r: max(abs(r['roll']), abs(r['pitch'])) > 0.6)}"
    )
    print(
        "state "
        f"min_com_z={min_com_z:.3f} "
        f"max_abs_roll={max_abs_roll:.3f} "
        f"max_abs_pitch={max_abs_pitch:.3f} "
        f"max_xy_drift={max_xy_drift:.3f}"
    )
    print(
        "contact "
        f"stance_samples={len(stance_rows)} "
        f"zero_stance_samples={len(zero_stance)} "
        f"first_zero_stance={zero_stance[0]['time'] if zero_stance else None}"
    )
    if has_columns(rows, "left_foot_z", "right_foot_z"):
        for side in ("left", "right"):
            stats = side_swing_stats(walking, side)
            print(
                f"{side}_swing "
                f"mode_fraction={stats['mode_swing_fraction']:.3f} "
                f"scheduled_fraction={stats['scheduled_swing_fraction']:.3f} "
                f"samples={stats['swing_samples']} "
                f"max_foot_z={stats['max_swing_foot_z']:.3f} "
                f"p95_foot_z={stats['p95_swing_foot_z']:.3f} "
                f"stance_foot_z=[{stats['min_stance_foot_z']:.3f},{stats['max_stance_foot_z']:.3f}]"
            )
    if has_columns(rows, "left_cm_active", "right_cm_active"):
        zero_active = [
            row for row in walking
            if row["left_cm_active"] != 1 and row["right_cm_active"] != 1
        ]
        early_rows = [
            row for row in walking
            if row.get("left_cm_early", 0) == 1 or row.get("right_cm_early", 0) == 1
        ]
        late_rows = [
            row for row in walking
            if row.get("left_cm_late", 0) == 1 or row.get("right_cm_late", 0) == 1
        ]
        liftoff_hold_rows = [
            row for row in walking
            if row.get("left_cm_liftoff_hold", 0) == 1
            or row.get("right_cm_liftoff_hold", 0) == 1
        ]
        print(
            "contact_manager "
            f"zero_active_samples={len(zero_active)} "
            f"first_zero_active={zero_active[0]['time'] if zero_active else None} "
            f"early_samples={len(early_rows)} "
            f"late_samples={len(late_rows)} "
            f"liftoff_hold_samples={len(liftoff_hold_rows)}"
        )
    if has_columns(rows, "gait_recovery_hold"):
        recovery_rows = [row for row in walking if row["gait_recovery_hold"] == 1]
        print(
            "gait_recovery "
            f"samples={len(recovery_rows)} "
            f"first={recovery_rows[0]['time'] if recovery_rows else None} "
            f"last={recovery_rows[-1]['time'] if recovery_rows else None}"
        )
    if force_errors:
        print(
            "force_error "
            f"mean={statistics.mean(force_errors):.1f} "
            f"p95={percentile(force_errors, 0.95):.1f} "
            f"max={max(force_errors):.1f}"
        )
    if moment_errors:
        print(
            "net_moment_error "
            f"mean={statistics.mean(moment_errors):.1f} "
            f"p95={percentile(moment_errors, 0.95):.1f} "
            f"max={max(moment_errors):.1f}"
        )

    print("snapshots")
    for target in (3.5, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 26.0):
        if target > rows[-1]["time"]:
            continue
        near = min(rows, key=lambda row: abs(row["time"] - target))
        line = (
            f"  t={near['time']:.2f} "
            f"z={near['com_z']:.3f} "
            f"roll={near['roll']:.3f} pitch={near['pitch']:.3f} "
            f"yaw={near['yaw']:.3f} "
            f"fz={near['total_actual_fz']:.1f}/{near['total_desired_fz']:.1f} "
            f"modes={int(near['left_mode'])},{int(near['right_mode'])} "
            f"fn={near['left_fn']:.1f},{near['right_fn']:.1f}"
        )
        if has_columns(rows, "left_cm_active", "right_cm_active"):
            line += (
                f" cm_active={int(near['left_cm_active'])},{int(near['right_cm_active'])}"
                f" cm_sched={int(near['left_cm_scheduled'])},{int(near['right_cm_scheduled'])}"
                f" alpha={near['left_cm_alpha']:.2f},{near['right_cm_alpha']:.2f}"
            )
            if has_columns(rows, "left_cm_liftoff_hold", "right_cm_liftoff_hold"):
                line += (
                    f" hold={int(near['left_cm_liftoff_hold'])},"
                    f"{int(near['right_cm_liftoff_hold'])}"
                )
        if has_columns(rows, "target_y", "target_z", "target_roll", "target_pitch"):
            line += (
                f" target_yz={near['target_y']:.3f},{near['target_z']:.3f}"
                f" target_rp={near['target_roll']:.3f},{near['target_pitch']:.3f}"
            )
        if has_columns(rows, "gait_recovery_hold"):
            line += (
                f" recovery={int(near['gait_recovery_hold'])}"
                f"({near.get('gait_recovery_hold_time', 0.0):.2f})"
            )
        if has_columns(rows, "total_actual_mx", "total_desired_mx"):
            line += (
                f" Mx={near['total_actual_mx']:.1f}/{near['total_desired_mx']:.1f}"
                f" My={near['total_actual_my']:.1f}/{near['total_desired_my']:.1f}"
            )
        print(line)


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
        "logs/debug/headless_telemetry/walking_telemetry.csv"
    )
    summarize(path)


if __name__ == "__main__":
    main()
