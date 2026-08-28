#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/grasp_bottle.sh                                    # preview: head/locate/plan, zero motion
  ./scripts/grasp_bottle.sh --execute --safety-acknowledged    # full flow: head down -> locate -> reach -> grip

Depth-guided left-arm bottle grasp (head camera at pitch -30 deg). Preview is
the default. A real run requires both gates, a clear workspace, an operator
watching, and the physical emergency stop in reach. Fail-closed: any stage
failure stops in place; nothing retries or auto-releases. See
examples/arm/depth_guided_grasp.py for details.
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
runner="${project_root}/examples/arm/depth_guided_grasp.py"

for required in "$python_bin" "$runner"; do
  [[ -e "$required" ]] || { echo "Error: required runtime file is missing: $required" >&2; exit 2; }
done

# 从干净的加载器路径开始：继承的 LD_LIBRARY_PATH 若暴露系统
# libssl 会弄坏 conda 的 cv2（libcurl 需要 OPENSSL_3.2.0）。
unset LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROBOT_ID="${ROBOT_ID:-274}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>255</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"
export PYTHONPATH="/home/ubuntu/Documents/AutolifeRobotSDK/sdk_python:/home/ubuntu/Documents/AutolifeRobotVision/src:${project_root}${PYTHONPATH:+:${PYTHONPATH}}"

exec "$python_bin" "$runner" "$@"
