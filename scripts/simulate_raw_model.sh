#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage: simulate_raw_model.sh <m|g|h> [--] [simulate_args...]

Launch MuJoCo simulate with a raw XML model.

Robot id:
  m  MIT humanoid         -> models/mit_humanoid/scene.xml
  g  Unitree G1           -> models/unitree_robots/g1/scene_23dof.xml
  h  Unitree H1           -> models/unitree_robots/h1/scene.xml

Options:
  -h, --help  Show this help message.

Any extra arguments after -- are forwarded to simulate.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
simulate_bin="${HOME}/.local/mujoco/bin/simulate"
xml_path=""
robot_id=""

if [[ ! -x "${simulate_bin}" ]]; then
    echo "Error: MuJoCo simulate not found at ${simulate_bin}" >&2
    exit 1
fi

extra_args=()

if [[ $# -eq 0 ]]; then
    echo "Error: missing robot id (expected m, g, or h)" >&2
    usage >&2
    exit 1
fi

case "$1" in
    -h|--help)
        usage
        exit 0
        ;;
    m|M)
        robot_id="m"
        shift
        ;;
    g|G)
        robot_id="g"
        shift
        ;;
    h|H)
        robot_id="h"
        shift
        ;;
    -*)
        echo "Error: unknown option: $1" >&2
        usage >&2
        exit 1
        ;;
    *)
        echo "Error: expected robot id m, g, or h; got: $1" >&2
        usage >&2
        exit 1
        ;;
esac

if [[ $# -gt 0 && "$1" == "--" ]]; then
    shift
fi

extra_args=("$@")

case "${robot_id}" in
    m)
        xml_path="${repo_root}/models/mit_humanoid/scene.xml"
        ;;
    g)
        xml_path="${repo_root}/models/unitree_robots/g1/scene_23dof.xml"
        ;;
    h)
        xml_path="${repo_root}/models/unitree_robots/h1/scene.xml"
        ;;
    *)
        echo "Error: unsupported robot id: ${robot_id}" >&2
        exit 1
        ;;
esac

echo "XML path: ${xml_path}" >&2

if [[ ! -f "${xml_path}" ]]; then
    echo "Error: XML file not found: ${xml_path}" >&2
    exit 1
fi

if [[ ${#extra_args[@]} -eq 0 ]]; then
    exec "${simulate_bin}" "${xml_path}"
fi

exec "${simulate_bin}" "${xml_path}" "${extra_args[@]}"
