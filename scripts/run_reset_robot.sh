#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

export PYTHONPATH="/home/ubuntu/Documents/AutolifeRobotSDK/sdk_python${PYTHONPATH:+:${PYTHONPATH}}"
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROBOT_ID="$(hostname | sed -e "s/.*-//")"   # 机器编号按主机名推导，一律覆盖
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>255</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"

exec /home/ubuntu/miniconda3/bin/conda run --no-capture-output -n robot_env \
  python "${SCRIPT_DIR}/reset_robot.py" "$@"
