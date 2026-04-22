#!/usr/bin/env bash

set -euo pipefail

# Usage:
#   ./scripts/run_standing_debug.sh
#   ./scripts/run_standing_debug.sh -l logs/debug/standing_mpc/standing_mpc_debug_YYYYMMDD_HHMMSS.json
#   ./scripts/run_standing_debug.sh -s
#   ./scripts/run_standing_debug.sh -c
#   ./scripts/run_standing_debug.sh -r -n 80
#
# Options:
#   -l PATH  Use a specific standing MPC debug JSON for both tools.
#   -s       Skip srb_reconstruct.py.
#   -c       Skip stand_contact_probe.
#   -r       Also run stand_rh_probe.
#   -n N     Receding-horizon rollout steps; implies -r.
#   -h       Show this help and exit.

usage() {
    cat <<'EOF'
Usage: run_standing_debug.sh [-l LOG] [-s] [-c] [-r] [-n STEPS] [-h]

Run the standing MPC reconstruction plotter and contact probe in sequence.

Options:
  -l PATH  Use a specific standing MPC debug JSON for both tools.
  -s       Skip srb_reconstruct.py.
  -c       Skip stand_contact_probe.
  -r       Also run stand_rh_probe.
  -n N     Receding-horizon rollout steps; implies -r.
  -h       Show this help and exit.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

python_bin="${PYTHON:-python}"
srb_script="${repo_root}/test/standing_debug/srb_reconstruct.py"
probe_bin="${repo_root}/build/test/standing_debug/stand_contact_probe"
rh_bin="${repo_root}/build/test/standing_debug/stand_rh_probe"

run_srb=1
run_probe=1
run_rh=0
rh_steps=60
log_path=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        -l)
            if [[ $# -lt 2 ]]; then
                echo "Error: -l requires a path" >&2
                exit 1
            fi
            log_path="$2"
            shift 2
            ;;
        -s)
            run_srb=0
            shift
            ;;
        -c)
            run_probe=0
            shift
            ;;
        -r)
            run_rh=1
            shift
            ;;
        -n)
            if [[ $# -lt 2 ]]; then
                echo "Error: -n requires a positive integer" >&2
                exit 1
            fi
            rh_steps="$2"
            run_rh=1
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

if ! command -v "${python_bin}" >/dev/null 2>&1; then
    echo "Error: python executable not found: ${python_bin}" >&2
    exit 1
fi

if [[ ! -f "${srb_script}" ]]; then
    echo "Error: missing srb_reconstruct.py at ${srb_script}" >&2
    exit 1
fi

if [[ ${run_probe} -eq 1 && ! -x "${probe_bin}" ]]; then
    echo "Error: stand_contact_probe binary not found or not executable at ${probe_bin}" >&2
    echo "Build it first with: cmake --build build --target stand_contact_probe" >&2
    exit 1
fi

if [[ ${run_rh} -eq 1 && ! -x "${rh_bin}" ]]; then
    echo "Error: stand_rh_probe binary not found or not executable at ${rh_bin}" >&2
    echo "Build it first with: cmake --build build --target stand_rh_probe" >&2
    exit 1
fi

if [[ -n "${log_path}" && "${log_path}" != /* ]]; then
    log_path="${repo_root}/${log_path}"
fi

if [[ ${run_srb} -eq 1 ]]; then
    if [[ -n "${log_path}" ]]; then
        "${python_bin}" "${srb_script}" "${log_path}"
    else
        "${python_bin}" "${srb_script}"
    fi
fi

if [[ ${run_probe} -eq 1 ]]; then
    if [[ -n "${log_path}" ]]; then
        "${probe_bin}" "${log_path}"
    else
        "${probe_bin}"
    fi
fi

if [[ ${run_rh} -eq 1 ]]; then
    if [[ -n "${log_path}" ]]; then
        "${rh_bin}" -n "${rh_steps}" "${log_path}"
    else
        "${rh_bin}" -n "${rh_steps}"
    fi
fi
