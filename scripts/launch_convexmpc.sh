#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

default_main_bin="${repo_root}/build/apps/main"
default_dashboard_script="${repo_root}/dashboard/app.py"
default_dashboard_port=8001

main_bin="${default_main_bin}"
python_bin="${CONVEXMPC_PYTHON:-${PYTHON:-python3}}"
dashboard_script=""
dashboard_port="${CONVEXMPC_DASHBOARD_PORT:-${default_dashboard_port}}"
dashboard_explicit=false
dashboard_enabled=true
robot=""
viewer=""

usage() {
    cat <<'EOF'
Usage: launch_convexmpc.sh [options] <robot> <viewer>

Launch the ConvexMPC control binary and, when available, a Python dashboard.

Positional arguments:
  robot   m, g, or h
  viewer  y or n

Options:
  --main-bin PATH          Override the C++ executable path.
  --dashboard-script PATH   Python dashboard entrypoint to run.
  --dashboard-port PORT    Dashboard port to use, or the first free port at/above it.
  --python PYTHON          Python interpreter for the dashboard.
  --no-dashboard           Skip the dashboard even if one is configured.
  -h, --help               Show this help and exit.

Environment:
  CONVEXMPC_PYTHON         Default Python interpreter if --python is not provided.
  CONVEXMPC_DASHBOARD_SCRIPT  Default dashboard entrypoint if --dashboard-script is not provided.
  CONVEXMPC_DASHBOARD_PORT   Default dashboard port if --dashboard-port is not provided.
EOF
}

pick_free_port() {
    local port="$1"
    while lsof -nP -iTCP:"${port}" -sTCP:LISTEN >/dev/null 2>&1; do
        port=$((port + 1))
    done
    printf '%s\n' "${port}"
}

clear_stale_shared_memory() {
    "${python_bin}" - "${CONVEXMPC_SHM_NAME}" <<'PY'
import sys
from multiprocessing import shared_memory

name = sys.argv[1]
try:
    shm = shared_memory.SharedMemory(name=name)
except FileNotFoundError:
    raise SystemExit(0)

try:
    shm.close()
finally:
    try:
        shm.unlink()
    except FileNotFoundError:
        pass
PY
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --main-bin)
            if [[ $# -lt 2 ]]; then
                echo "Error: --main-bin requires a path" >&2
                exit 1
            fi
            main_bin="$2"
            shift 2
            ;;
        --dashboard-script)
            if [[ $# -lt 2 ]]; then
                echo "Error: --dashboard-script requires a path" >&2
                exit 1
            fi
            dashboard_script="$2"
            dashboard_explicit=true
            shift 2
            ;;
        --dashboard-port)
            if [[ $# -lt 2 ]]; then
                echo "Error: --dashboard-port requires a port" >&2
                exit 1
            fi
            dashboard_port="$2"
            shift 2
            ;;
        --python)
            if [[ $# -lt 2 ]]; then
                echo "Error: --python requires a path" >&2
                exit 1
            fi
            python_bin="$2"
            shift 2
            ;;
        --no-dashboard)
            dashboard_enabled=false
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            break
            ;;
        -*)
            echo "Error: unknown option: $1" >&2
            usage >&2
            exit 1
            ;;
        *)
            if [[ -z "${robot}" ]]; then
                robot="$1"
            elif [[ -z "${viewer}" ]]; then
                viewer="$1"
            else
                echo "Error: unexpected extra argument: $1" >&2
                usage >&2
                exit 1
            fi
            shift
            ;;
    esac
done

if [[ -n "${dashboard_script}" && "${dashboard_script}" != /* ]]; then
    dashboard_script="${repo_root}/${dashboard_script}"
fi

if [[ -z "${dashboard_script}" && -n "${CONVEXMPC_DASHBOARD_SCRIPT:-}" ]]; then
    dashboard_script="${CONVEXMPC_DASHBOARD_SCRIPT}"
    dashboard_explicit=true
fi

if ! [[ "${dashboard_port}" =~ ^[0-9]+$ ]]; then
    echo "Error: dashboard port must be a number: ${dashboard_port}" >&2
    exit 1
fi
dashboard_port="$(pick_free_port "${dashboard_port}")"

if ! command -v "${python_bin}" >/dev/null 2>&1; then
    echo "Error: python executable not found: ${python_bin}" >&2
    exit 1
fi

export CONVEXMPC_SHM_NAME="${CONVEXMPC_SHM_NAME:-convexmpc_dashboard_state}"
clear_stale_shared_memory

if [[ -z "${robot}" || -z "${viewer}" ]]; then
    usage >&2
    exit 1
fi

case "${robot}" in
    m|M|g|G|h|H)
        ;;
    *)
        echo "Error: robot must be one of m, g, or h" >&2
        exit 1
        ;;
esac

case "${viewer}" in
    y|Y|n|N)
        ;;
    *)
        echo "Error: viewer must be y or n" >&2
        exit 1
        ;;
esac

if [[ ! -x "${main_bin}" ]]; then
    echo "Error: main executable not found or not executable: ${main_bin}" >&2
    exit 1
fi

export CONVEXMPC_SHM_NAME="${CONVEXMPC_SHM_NAME:-convexmpc_dashboard_state}"
export CONVEXMPC_ROBOT="${robot}"
export CONVEXMPC_VIEWER="${viewer}"
export CONVEXMPC_DASHBOARD_PORT="${dashboard_port}"
export CONVEXMPC_DASHBOARD_OPEN_BROWSER="${CONVEXMPC_DASHBOARD_OPEN_BROWSER:-1}"

if [[ "${dashboard_enabled}" == true && -z "${dashboard_script}" ]]; then
    if [[ -f "${default_dashboard_script}" ]]; then
        dashboard_script="${default_dashboard_script}"
    fi
fi

if [[ "${dashboard_enabled}" == true && -n "${dashboard_script}" && ! -f "${dashboard_script}" ]]; then
    if [[ "${dashboard_explicit}" == true ]]; then
        echo "Error: dashboard script not found: ${dashboard_script}" >&2
        exit 1
    fi
    dashboard_script=""
fi

if [[ "${dashboard_enabled}" == false || -z "${dashboard_script}" ]]; then
    if [[ -n "${dashboard_script}" ]]; then
        echo "[launch] dashboard disabled; launching main only" >&2
    else
        echo "[launch] no dashboard script found; launching main only" >&2
    fi
    exec "${main_bin}" "${robot}" "${viewer}"
fi

main_pid=""
dashboard_pid=""

cleanup() {
    set +e
    if [[ -n "${dashboard_pid}" ]]; then
        kill "${dashboard_pid}" >/dev/null 2>&1
    fi
    if [[ -n "${main_pid}" ]]; then
        kill "${main_pid}" >/dev/null 2>&1
    fi
}

trap cleanup EXIT INT TERM

echo "[launch] main: ${main_bin} ${robot} ${viewer}"
echo "[launch] dashboard: ${python_bin} ${dashboard_script} --host 127.0.0.1 --port ${dashboard_port}"

"${python_bin}" "${dashboard_script}" --host 127.0.0.1 --port "${dashboard_port}" &
dashboard_pid=$!

"${main_bin}" "${robot}" "${viewer}" &
main_pid=$!

set +e
wait "${main_pid}"
main_status=$?
set -e

if [[ -n "${dashboard_pid}" ]]; then
    kill "${dashboard_pid}" >/dev/null 2>&1 || true
    set +e
    wait "${dashboard_pid}" >/dev/null 2>&1
    set -e
fi

exit "${main_status}"
