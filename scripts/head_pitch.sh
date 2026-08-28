#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/head_pitch.sh                 # preview: read and print current neck angles
  ./scripts/head_pitch.sh --pitch DEG     # dry run: validate and print the payload
  ./scripts/head_pitch.sh --pitch DEG --execute --safety-acknowledged

Bounded neck-pitch move (head up/down only). Preview is the default; a real
move requires both --execute and --safety-acknowledged. Positive pitch is
head-up. See examples/arm/head_pitch.py --help for details.
EOF
}

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
script_dir="$(cd -- "$(dirname -- "$script_path")" && pwd)"
if [[ "$(basename -- "$script_dir")" == "scripts" ]]; then
  project_root="$(cd -- "${script_dir}/.." && pwd)"
else
  project_root="$(cd -- "${script_dir}/../.." && pwd)"
fi
python_bin="/home/ubuntu/miniconda3/envs/robot_env/bin/python"
runner="${project_root}/examples/arm/head_pitch.py"

for required in "$python_bin" "$runner"; do
  [[ -e "$required" ]] || { echo "Error: required runtime file is missing: $required" >&2; exit 2; }
done

source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROBOT_ID="${ROBOT_ID:-274}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>255</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"
export PYTHONPATH="/home/ubuntu/Documents/AutolifeRobotSDK/sdk_python:${project_root}${PYTHONPATH:+:${PYTHONPATH}}"

exec "$python_bin" "$runner" "$@"
