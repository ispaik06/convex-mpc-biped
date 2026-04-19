#!/usr/bin/env bash

set -euo pipefail

usage() {
    cat <<'EOF'
Usage: simulate_raw_model.sh [-m|--xml XML_PATH] [--] [simulate_args...]

Launch MuJoCo simulate with a raw XML model.

Options:
  -m, --xml   Path to the MuJoCo XML file. Defaults to models/mit_humanoid/scene.xml.
  -h, --help  Show this help message.

Any extra arguments after -- are forwarded to simulate.
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
simulate_bin="${HOME}/.local/mujoco/bin/simulate"
xml_path="${repo_root}/models/mit_humanoid/scene.xml"

if [[ ! -x "${simulate_bin}" ]]; then
    echo "Error: MuJoCo simulate not found at ${simulate_bin}" >&2
    exit 1
fi

extra_args=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        -m|--xml)
            if [[ $# -lt 2 ]]; then
                echo "Error: --xml requires a path" >&2
                exit 1
            fi
            xml_path="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            extra_args=("$@")
            break
            ;;
        *)
            extra_args+=("$1")
            shift
            ;;
    esac
done

if [[ ! -f "${xml_path}" ]]; then
    echo "Error: XML file not found: ${xml_path}" >&2
    exit 1
fi

exec "${simulate_bin}" "${xml_path}" "${extra_args[@]}"
