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
readonly wrench_reconstruction_script="${repo_root}/test/standing_debug/wrench_reconstruction.py"
readonly probe_bin="${repo_root}/build/test/standing_debug/stand_contact_probe"
readonly wrench_mapping_bin="${repo_root}/build/test/standing_debug/wrench_mapping_probe"
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

if [[ ! -f "${wrench_reconstruction_script}" ]]; then
    echo "Error: missing wrench_reconstruction.py at ${wrench_reconstruction_script}" >&2
    exit 1
fi

if [[ ! -x "${probe_bin}" ]]; then
    echo "Error: stand_contact_probe binary not found or not executable at ${probe_bin}" >&2
    echo "Build it first with: cmake --build build --target stand_contact_probe" >&2
    exit 1
fi

if [[ ! -x "${wrench_mapping_bin}" ]]; then
    echo "Error: wrench_mapping_probe binary not found or not executable at ${wrench_mapping_bin}" >&2
    echo "Build it first with: cmake --build build --target wrench_mapping_probe" >&2
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

tmp_dir="$(mktemp -d "${TMPDIR:-/tmp}/mpc_debug.XXXXXX")"
trap 'rm -rf "${tmp_dir}"' EXIT

extract_prefix_value() {
    local output_file="$1"
    local prefix="$2"
    awk -v prefix="${prefix}" '
        index($0, prefix) == 1 {
            value = substr($0, length(prefix) + 1)
        }
        END {
            if (value != "") {
                print value
            }
        }
    ' "${output_file}"
}

file_was_generated() {
    local path="$1"
    [[ -n "${path}" && -s "${path}" ]]
}

print_failure_tail() {
    local output_file="$1"
    local lines
    lines="$(grep -Ei "error|failed|exception|traceback|not found|skipped" "${output_file}" | tail -n 5 || true)"
    if [[ -z "${lines}" ]]; then
        lines="$(tail -n 5 "${output_file}" || true)"
    fi
    if [[ -n "${lines}" ]]; then
        echo "${lines}" | sed 's/^/    /'
    fi
}

summarize_srb_reconstruction() {
    local output_file="$1"
    local plot
    plot="$(extract_prefix_value "${output_file}" "saved plot: ")"
    if file_was_generated "${plot}"; then
        echo "[OK] SRB reconstruction: plot generated"
        echo "     plot: ${plot}"
        return 0
    fi

    echo "[FAIL] SRB reconstruction: expected plot was not generated"
    print_failure_tail "${output_file}"
    return 1
}

summarize_contact_probe() {
    local output_file="$1"
    local report csv plot
    report="$(extract_prefix_value "${output_file}" "report: ")"
    csv="$(extract_prefix_value "${output_file}" "csv: ")"
    plot="$(extract_prefix_value "${output_file}" "plot: ")"
    if file_was_generated "${report}" && file_was_generated "${csv}" && file_was_generated "${plot}"; then
        echo "[OK] Contact probe: report, CSV, and plot generated"
        echo "     report: ${report}"
        echo "     csv: ${csv}"
        echo "     plot: ${plot}"
        return 0
    fi

    echo "[FAIL] Contact probe: expected artifacts were not generated"
    [[ -n "${report}" ]] && echo "     report: ${report}"
    [[ -n "${csv}" ]] && echo "     csv: ${csv}"
    [[ -n "${plot}" ]] && echo "     plot: ${plot}"
    print_failure_tail "${output_file}"
    return 1
}

summarize_wrench_reconstruction() {
    local output_file="$1"
    local report csv plot skip
    report="$(extract_prefix_value "${output_file}" "wrench reconstruction report: ")"
    csv="$(extract_prefix_value "${output_file}" "wrench reconstruction csv: ")"
    plot="$(extract_prefix_value "${output_file}" "wrench reconstruction plot: ")"
    skip="$(extract_prefix_value "${output_file}" "wrench reconstruction: skipped plot ")"

    if [[ -n "${skip}" ]]; then
        if file_was_generated "${report}"; then
            echo "[SKIP] Wrench reconstruction: report generated, plot/CSV skipped"
            echo "       reason: ${skip}"
            echo "       report: ${report}"
            return 0
        fi
        echo "[FAIL] Wrench reconstruction: skipped but report was not generated"
        print_failure_tail "${output_file}"
        return 1
    fi

    if file_was_generated "${report}" && file_was_generated "${csv}" && file_was_generated "${plot}"; then
        echo "[OK] Wrench reconstruction: report, CSV, and plot generated"
        echo "     report: ${report}"
        echo "     csv: ${csv}"
        echo "     plot: ${plot}"
        return 0
    fi

    echo "[FAIL] Wrench reconstruction: expected artifacts were not generated"
    [[ -n "${report}" ]] && echo "     report: ${report}"
    [[ -n "${csv}" ]] && echo "     csv: ${csv}"
    [[ -n "${plot}" ]] && echo "     plot: ${plot}"
    print_failure_tail "${output_file}"
    return 1
}

summarize_receding_horizon() {
    local output_file="$1"
    local report csv states_plot wrench_plot metrics_plot
    report="$(extract_prefix_value "${output_file}" "report: ")"
    csv="$(extract_prefix_value "${output_file}" "csv: ")"
    states_plot="$(extract_prefix_value "${output_file}" "states plot: ")"
    wrench_plot="$(extract_prefix_value "${output_file}" "wrench plot: ")"
    metrics_plot="$(extract_prefix_value "${output_file}" "metrics plot: ")"
    if file_was_generated "${report}" && file_was_generated "${csv}" &&
       file_was_generated "${states_plot}" && file_was_generated "${wrench_plot}" &&
       file_was_generated "${metrics_plot}"; then
        echo "[OK] Receding horizon: report, CSV, and plots generated"
        echo "     report: ${report}"
        echo "     csv: ${csv}"
        echo "     states plot: ${states_plot}"
        echo "     wrench plot: ${wrench_plot}"
        echo "     metrics plot: ${metrics_plot}"
        return 0
    fi

    echo "[FAIL] Receding horizon: expected artifacts were not generated"
    [[ -n "${report}" ]] && echo "     report: ${report}"
    [[ -n "${csv}" ]] && echo "     csv: ${csv}"
    [[ -n "${states_plot}" ]] && echo "     states plot: ${states_plot}"
    [[ -n "${wrench_plot}" ]] && echo "     wrench plot: ${wrench_plot}"
    [[ -n "${metrics_plot}" ]] && echo "     metrics plot: ${metrics_plot}"
    print_failure_tail "${output_file}"
    return 1
}

run_analysis() {
    local key="$1"
    local label="$2"
    local summarizer="$3"
    shift 3

    local output_file="${tmp_dir}/${key}.log"
    if "$@" > "${output_file}" 2>&1; then
        "${summarizer}" "${output_file}"
        return $?
    fi

    local status=$?
    echo "[FAIL] ${label}: command exited with status ${status}"
    print_failure_tail "${output_file}"
    return "${status}"
}

if [[ "${selection_mode}" == "all-latest" ]]; then
    echo "MPC debug log: ${log_path} (latest across standing_mpc and walking_mpc)"
elif [[ "${selection_mode}" == "standing" || "${selection_mode}" == "walking" ]]; then
    echo "MPC debug log: ${log_path} (latest ${selection_mode})"
else
    echo "MPC debug log: ${log_path}"
fi
echo "Running MPC debug analyses..."

overall_status=0
run_analysis "srb_reconstruction" \
             "SRB reconstruction" \
             "summarize_srb_reconstruction" \
             "${python_bin}" "${srb_script}" "${log_path}" || overall_status=1
run_analysis "contact_probe" \
             "Contact probe" \
             "summarize_contact_probe" \
             "${probe_bin}" "${log_path}" || overall_status=1
run_analysis "wrench_reconstruction" \
             "Wrench reconstruction" \
             "summarize_wrench_reconstruction" \
             "${python_bin}" "${wrench_reconstruction_script}" "${log_path}" || overall_status=1
run_analysis "receding_horizon" \
             "Receding horizon" \
             "summarize_receding_horizon" \
             "${rh_bin}" -n "${rh_steps}" "${log_path}" || overall_status=1

if [[ "${overall_status}" -eq 0 ]]; then
    echo "MPC debug analyses completed."
else
    echo "MPC debug analyses completed with failures."
fi

exit "${overall_status}"
