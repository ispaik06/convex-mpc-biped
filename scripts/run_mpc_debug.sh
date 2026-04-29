#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage: run_mpc_debug.sh -n STEPS (--standing | --walking | --all-latest | -l LOG) [-h]

Run MPC debug post-processing for a standing or walking JSON log.

Options:
  -l PATH        Use a specific MPC debug JSON.
  --standing     Use the latest standing_mpc log.
  --walking      Use the latest walking_mpc log.
  --all-latest   Use the newest log across both directories.
  -n STEPS       Receding-horizon rollout steps. Required.
  -h, --help     Show this help and exit.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

python_bin="${PYTHON:-python}"
readonly srb_script="${repo_root}/test/standing_debug/srb_reconstruct.py"
readonly probe_bin="${repo_root}/build/test/standing_debug/stand_contact_probe"
readonly rh_bin="${repo_root}/build/test/standing_debug/stand_rh_probe"

rh_steps=""
log_path=""
selection_mode=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        -l)
            if [[ $# -lt 2 ]]; then
                echo "Error: -l requires a path" >&2
                exit 1
            fi
            if [[ -n "${selection_mode}" || -n "${log_path}" ]]; then
                echo "Error: choose exactly one of -l, --standing, --walking, or --all-latest" >&2
                exit 1
            fi
            log_path="$2"
            selection_mode="explicit"
            shift 2
            ;;
        --standing)
            if [[ -n "${selection_mode}" || -n "${log_path}" ]]; then
                echo "Error: choose exactly one of -l, --standing, --walking, or --all-latest" >&2
                exit 1
            fi
            selection_mode="standing"
            shift
            ;;
        --walking)
            if [[ -n "${selection_mode}" || -n "${log_path}" ]]; then
                echo "Error: choose exactly one of -l, --standing, --walking, or --all-latest" >&2
                exit 1
            fi
            selection_mode="walking"
            shift
            ;;
        --all-latest)
            if [[ -n "${selection_mode}" || -n "${log_path}" ]]; then
                echo "Error: choose exactly one of -l, --standing, --walking, or --all-latest" >&2
                exit 1
            fi
            selection_mode="all-latest"
            shift
            ;;
        -n)
            if [[ $# -lt 2 ]]; then
                echo "Error: -n requires a positive integer" >&2
                exit 1
            fi
            rh_steps="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Error: unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

if [[ -z "${rh_steps}" ]]; then
    echo "Error: -n STEPS is required" >&2
    usage >&2
    exit 1
fi

if [[ ! "${rh_steps}" =~ ^[0-9]+$ ]] || [[ "${rh_steps}" -le 0 ]]; then
    echo "Error: -n requires a positive integer" >&2
    exit 1
fi

if [[ -z "${selection_mode}" ]]; then
    echo "Error: choose exactly one of -l, --standing, --walking, or --all-latest" >&2
    usage >&2
    exit 1
fi

if ! command -v "${python_bin}" >/dev/null 2>&1; then
    echo "Error: python executable not found: ${python_bin}" >&2
    exit 1
fi

if [[ ! -f "${srb_script}" ]]; then
    echo "Error: missing srb_reconstruct.py at ${srb_script}" >&2
    exit 1
fi

if [[ ! -x "${probe_bin}" ]]; then
    echo "Error: stand_contact_probe binary not found or not executable at ${probe_bin}" >&2
    echo "Build it first with: cmake --build build --target stand_contact_probe" >&2
    exit 1
fi

if [[ ! -x "${rh_bin}" ]]; then
    echo "Error: stand_rh_probe binary not found or not executable at ${rh_bin}" >&2
    echo "Build it first with: cmake --build build --target stand_rh_probe" >&2
    exit 1
fi

if [[ -n "${log_path}" && "${log_path}" != /* ]]; then
    log_path="${repo_root}/${log_path}"
fi

select_latest_log() {
    local mode="$1"
    "${python_bin}" - "${repo_root}" "${mode}" <<'PY'
import sys
from pathlib import Path

root = Path(sys.argv[1])
mode = sys.argv[2]
standing_dir = root / "logs" / "debug" / "standing_mpc"
walking_dir = root / "logs" / "debug" / "walking_mpc"

def latest_in_dir(directory: Path, prefix: str) -> list[Path]:
    if not directory.exists():
        return []
    return [path for path in directory.glob(f"{prefix}*.json") if path.is_file()]

paths = []
if mode == "standing":
    paths = latest_in_dir(standing_dir, "standing_mpc_debug_")
elif mode == "walking":
    paths = latest_in_dir(walking_dir, "walking_mpc_debug_")
elif mode == "all-latest":
    paths = latest_in_dir(standing_dir, "standing_mpc_debug_") + latest_in_dir(
        walking_dir, "walking_mpc_debug_"
    )
else:
    raise SystemExit(f"Error: unsupported selection mode {mode}")

if not paths:
    if mode == "standing":
        raise SystemExit(f"Error: no standing MPC debug logs found in {standing_dir}")
    if mode == "walking":
        raise SystemExit(f"Error: no walking MPC debug logs found in {walking_dir}")
    raise SystemExit(
        f"Error: no MPC debug logs found in {standing_dir} or {walking_dir}"
    )

print(max(paths, key=lambda path: path.stat().st_mtime))
PY
}

if [[ "${selection_mode}" != "explicit" ]]; then
    log_path="$(select_latest_log "${selection_mode}")"
fi

if [[ ! -f "${log_path}" ]]; then
    echo "Error: log path does not exist: ${log_path}" >&2
    exit 1
fi

if [[ ! -s "${log_path}" ]]; then
    echo "Error: log path is empty: ${log_path}" >&2
    exit 1
fi

if [[ "${selection_mode}" == "all-latest" ]]; then
    echo "Using latest MPC log across standing_mpc and walking_mpc: ${log_path}"
elif [[ "${selection_mode}" == "standing" || "${selection_mode}" == "walking" ]]; then
    echo "Using latest ${selection_mode} MPC log: ${log_path}"
else
    echo "Using explicit MPC log: ${log_path}"
fi

"${python_bin}" "${srb_script}" "${log_path}"
"${probe_bin}" "${log_path}"
"${rh_bin}" -n "${rh_steps}" "${log_path}"
