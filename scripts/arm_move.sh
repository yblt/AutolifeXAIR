#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./scripts/arm_move.sh --arm left|right --direction up|down|left|right|forward|backward --distance METRES [--execute]
  ./scripts/arm_move.sh --arm left|right --axis x|y|z --angle SIGNED_DEGREES [--execute]
  ./scripts/arm_move.sh --arm left|right|both --gripper open|close [--execute]

Options:
  --arm         Arm/side: left or right; both is valid only for --gripper.
  --direction   Translation direction in the robot base frame.
  --distance    Positive translation distance in metres; maximum 0.30.
  --axis        Rotation axis in the robot base frame: x, y, or z.
  --angle       Signed rotation angle in degrees; non-zero and within [-20, 20].
  --gripper     Preview or command the selected gripper(s): open or close.
  --execute     Execute one physical command. Without it, only preview.
  -h, --help    Show this help.

Direction mapping (robot's perspective):
  forward/backward = +X/-X, left/right = +Y/-Y, up/down = +Z/-Z.

Rotation sign convention:
  Right-hand rule. Viewed from the positive axis toward the origin,
  a positive angle is counterclockwise and a negative angle is clockwise.

Safety:
  For arm motion, --execute confirms workspace clearance, emergency-stop
  access, and approval of the temporary relative-control policy. For grippers,
  --execute confirms pinch clearance and emergency-stop access.
EOF
}

fail() {
  echo "Error: $*" >&2
  echo "Run './scripts/arm_move.sh --help' for usage." >&2
  exit 2
}

need_value() {
  [[ $# -ge 2 ]] || fail "$1 requires a value"
}

is_finite_number() {
  [[ "$1" =~ ^[+-]?(([0-9]+([.][0-9]*)?)|([.][0-9]+))([eE][+-]?[0-9]+)?$ ]]
}

arm=""
direction=""
distance=""
axis=""
angle=""
gripper=""
execute=0
have_arm=0
have_direction=0
have_distance=0
have_axis=0
have_angle=0
have_gripper=0
have_execute=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --arm)
      [[ $have_arm -eq 0 ]] || fail "--arm may be supplied only once"
      need_value "$@"
      arm="$2"
      have_arm=1
      shift 2
      ;;
    --direction)
      [[ $have_direction -eq 0 ]] || fail "--direction may be supplied only once"
      need_value "$@"
      direction="$2"
      have_direction=1
      shift 2
      ;;
    --distance)
      [[ $have_distance -eq 0 ]] || fail "--distance may be supplied only once"
      need_value "$@"
      distance="$2"
      have_distance=1
      shift 2
      ;;
    --axis)
      [[ $have_axis -eq 0 ]] || fail "--axis may be supplied only once"
      need_value "$@"
      axis="$2"
      have_axis=1
      shift 2
      ;;
    --angle)
      [[ $have_angle -eq 0 ]] || fail "--angle may be supplied only once"
      need_value "$@"
      angle="$2"
      have_angle=1
      shift 2
      ;;
    --gripper)
      [[ $have_gripper -eq 0 ]] || fail "--gripper may be supplied only once"
      need_value "$@"
      gripper="$2"
      have_gripper=1
      shift 2
      ;;
    --execute)
      [[ $have_execute -eq 0 ]] || fail "--execute may be supplied only once"
      execute=1
      have_execute=1
      shift
      ;;
    *)
      fail "unknown argument: $1"
      ;;
  esac
done

[[ $have_arm -eq 1 ]] || fail "--arm is required"
[[ "$arm" == "left" || "$arm" == "right" || "$arm" == "both" ]] || fail "--arm must be left, right, or both"

translation_requested=0
rotation_requested=0
gripper_requested=$have_gripper
[[ $have_direction -eq 1 || $have_distance -eq 1 ]] && translation_requested=1
[[ $have_axis -eq 1 || $have_angle -eq 1 ]] && rotation_requested=1
operation_count=$((translation_requested + rotation_requested + gripper_requested))
[[ $operation_count -eq 1 ]] || fail "choose exactly one complete translation, rotation, or gripper form"

if [[ $gripper_requested -eq 1 ]]; then
  [[ "$gripper" == "open" || "$gripper" == "close" ]] || fail "--gripper must be open or close"
else
  [[ "$arm" == "left" || "$arm" == "right" ]] || fail "--arm both is valid only with --gripper"
  controller_args=(--side "$arm")

  if [[ $translation_requested -eq 1 ]]; then
    [[ $have_direction -eq 1 && $have_distance -eq 1 ]] || fail "translation requires both --direction and --distance"
    case "$direction" in
      up|down|left|right|forward|backward) ;;
      *) fail "--direction must be up, down, left, right, forward, or backward" ;;
    esac
    is_finite_number "$distance" || fail "--distance must be a finite number"
    awk -v value="$distance" 'BEGIN { exit !(value > 0 && value <= 0.30) }' || fail "--distance must be > 0 and <= 0.30 m"
    distance_magnitude="${distance#+}"

    dx=0
    dy=0
    dz=0
    case "$direction" in
      forward)  dx="$distance_magnitude" ;;
      backward) dx="-$distance_magnitude" ;;
      left)     dy="$distance_magnitude" ;;
      right)    dy="-$distance_magnitude" ;;
      up)       dz="$distance_magnitude" ;;
      down)     dz="-$distance_magnitude" ;;
    esac
    controller_args+=(--translate "$dx" "$dy" "$dz")
  else
    [[ $have_axis -eq 1 && $have_angle -eq 1 ]] || fail "rotation requires both --axis and --angle"
    [[ "$axis" == "x" || "$axis" == "y" || "$axis" == "z" ]] || fail "--axis must be x, y, or z"
    is_finite_number "$angle" || fail "--angle must be a finite number"
    awk -v value="$angle" 'BEGIN { exit !(value != 0 && value >= -20 && value <= 20) }' || fail "--angle must be non-zero and within [-20, 20] deg"
    controller_args+=(--rotate "$axis" "$angle")
  fi
fi

source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROBOT_ID="${ROBOT_ID:-274}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export PYTHONPATH="/home/ubuntu/Documents/AutolifeRobotSDK/sdk_python${PYTHONPATH:+:${PYTHONPATH}}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>255</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"

script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
script_dir="$(cd -- "$(dirname -- "$script_path")" && pwd)"

if [[ $gripper_requested -eq 1 ]]; then
  command=(/home/ubuntu/miniconda3/bin/conda run --no-capture-output -n robot_env \
    python "${script_dir}/control_gripper.py" --side "$arm" --action "$gripper")
  if [[ $execute -eq 1 ]]; then
    command+=(--execute --safety-acknowledged)
  fi
else
  command=(/usr/bin/python3 "${script_dir}/move_end_effector_relative.py" "${controller_args[@]}")
  if [[ $execute -eq 1 ]]; then
    command+=(--execute --safety-acknowledged --experimental-policy-approved)
  fi
fi

exec "${command[@]}"
