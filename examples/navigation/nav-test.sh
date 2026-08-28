#!/usr/bin/env bash
set -eo pipefail

# 整机可能更换（编号随之变化），只校验 AutoLife 机器人主机名前缀；
# 机器编号在下方从实际主机名推导。
EXPECTED_HOSTNAME_PATTERN="autolife-robot-*"
CONDA_SH="/home/ubuntu/miniconda3/etc/profile.d/conda.sh"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
TEB_SETUP="/home/ubuntu/Documents/teb/setup.bash"
ROBOT_ENV="/home/ubuntu/miniconda3/envs/robot_env"
TEST_PUB="${ROBOT_ENV}/lib/python3.12/site-packages/autolife_robot_gv/scripts/test_pub.py"
# nav-test.sh 的全部逻辑都在部署的仓库里，`git pull` 即可更新。
NAV_TEST_LIB="/home/ubuntu/Documents/AutolifeXAIR/examples/navigation"
NAVIGATE_SCRIPT="${NAV_TEST_LIB}/navigate_to_named_point.py"
ZERO_POSITION_SCRIPT="/home/ubuntu/Documents/AutolifeXAIR/examples/arm/play_zero_position.py"
MENU_SCRIPT="${NAV_TEST_LIB}/nav_test_menu.py"
RESET_SCRIPT="/home/ubuntu/Documents/AutolifeXAIR/scripts/reset_robot.py"

usage() {
    cat <<'EOF'
Usage:
  ./examples/navigation/nav-test.sh
  ./examples/navigation/nav-test.sh menu
  ./examples/navigation/nav-test.sh locate
  ./examples/navigation/nav-test.sh preview POINT [--feedback-timeout SECONDS]
  ./examples/navigation/nav-test.sh go POINT
  ./examples/navigation/nav-test.sh zero-position
  ./examples/navigation/nav-test.sh navigate POINT [navigate_to_named_point.py options]
  ./examples/navigation/nav-test.sh vendor
  ./examples/navigation/nav-test.sh help

Commands:
  menu      Start the slim interactive menu (default): locate, go, stop,
            maps, reset. Motion keeps preview + y/N confirmation.
  locate    Run the initial-localization flow directly: checklist, point
            selection, then publish initial_pose after confirmation.
  preview   Validate the active map and named point without publishing motion.
  go        Preview POINT, request interactive y/N safety confirmation, then
            navigate with a 120-second feedback timeout.
  zero-position
            Preview the configured whole-body zero-position action, then
            request interactive y/N safety confirmation before publishing.
  navigate  Run the guarded navigation helper. Physical motion still requires
            both --execute and --safety-acknowledged.
  vendor    Start the full vendor test_pub.py menu (map editing, SLAM mode,
            transitions, reset actions).
  help      Show this command structure.
EOF
}

fail() {
    printf 'nav-test: %s\n' "$*" >&2
    exit 1
}

command_name="${1:-menu}"
if [[ "$command_name" == "help" || "$command_name" == "--help" || "$command_name" == "-h" ]]; then
    usage
    exit 0
fi
if [[ $# -gt 0 ]]; then
    shift
fi

actual_hostname="$(hostname)"
if [[ "$actual_hostname" != $EXPECTED_HOSTNAME_PATTERN ]]; then
    fail "expected host matching ${EXPECTED_HOSTNAME_PATTERN}, got ${actual_hostname}"
fi

for required_file in "$CONDA_SH" "$ROS_SETUP" "$TEB_SETUP" "$TEST_PUB"; do
    [[ -f "$required_file" ]] || fail "required file not found: ${required_file}"
done
if [[ "$command_name" == "preview" || "$command_name" == "go" || "$command_name" == "navigate" ]]; then
    [[ -f "$NAVIGATE_SCRIPT" ]] || fail "required file not found: ${NAVIGATE_SCRIPT}"
fi
if [[ "$command_name" == "zero-position" ]]; then
    [[ -f "$ZERO_POSITION_SCRIPT" ]] || fail "required file not found: ${ZERO_POSITION_SCRIPT}"
fi
if [[ "$command_name" == "menu" || "$command_name" == "locate" ]]; then
    for required_file in "$MENU_SCRIPT" "$NAVIGATE_SCRIPT" "$RESET_SCRIPT"; do
        [[ -f "$required_file" ]] || fail "required file not found: ${required_file}"
    done
fi

# 先激活 Conda 再 source ROS，让 ros2cli 保住 ROS 的 Python 元数据。
source "$CONDA_SH"
conda activate robot_env
source "$ROS_SETUP"
source "$TEB_SETUP"

export ROS_DOMAIN_ID=0
export ROBOT_ID="${actual_hostname##*-}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export AMENT_PREFIX_PATH="${ROBOT_ENV}${AMENT_PREFIX_PATH:+:${AMENT_PREFIX_PATH}}"
export CMAKE_PREFIX_PATH="${ROBOT_ENV}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${ROBOT_ENV}/lib:/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export PYTHONPATH="${ROBOT_ENV}/lib/python3.12/site-packages${PYTHONPATH:+:${PYTHONPATH}}"
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>255</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

case "$command_name" in
    menu)
        [[ $# -eq 0 ]] || fail "menu does not accept arguments"
        exec "${ROBOT_ENV}/bin/python" "$MENU_SCRIPT"
        ;;
    locate)
        [[ $# -eq 0 ]] || fail "locate does not accept arguments"
        exec "${ROBOT_ENV}/bin/python" "$MENU_SCRIPT" locate
        ;;
    vendor)
        [[ $# -eq 0 ]] || fail "vendor does not accept arguments"
        exec "${ROBOT_ENV}/bin/python" "$TEST_PUB"
        ;;
    preview)
        [[ $# -gt 0 ]] || fail "preview requires an exact point name"
        for argument in "$@"; do
            case "$argument" in
                --execute|--safety-acknowledged)
                    fail "preview never accepts motion execution flags"
                    ;;
            esac
        done
        exec "${ROBOT_ENV}/bin/python" "$NAVIGATE_SCRIPT" "$@"
        ;;
    go)
        [[ $# -eq 1 ]] || fail "go requires exactly one point name"
        [[ -t 0 ]] || fail "go requires an interactive terminal for safety confirmation"
        point="$1"
        printf '%s\n' "=== Navigation preview ==="
        "${ROBOT_ENV}/bin/python" "$NAVIGATE_SCRIPT" \
            "$point" \
            --feedback-timeout 120
        printf '\nTarget: %s\n' "$point"
        printf '%s\n' 'Confirm that localization is correct, the route is clear,'
        printf '%s\n' 'the emergency stop is ready, and an operator is monitoring.'
        read -r -p 'Proceed with physical navigation? [y/N] ' answer
        case "$answer" in
            y|Y)
                exec "${ROBOT_ENV}/bin/python" "$NAVIGATE_SCRIPT" \
                    "$point" \
                    --feedback-timeout 120 \
                    --execute \
                    --safety-acknowledged
                ;;
            *)
                printf '%s\n' 'Navigation canceled by operator; no command was published.'
                ;;
        esac
        ;;
    zero-position)
        [[ $# -eq 0 ]] || fail "zero-position does not accept arguments"
        [[ -t 0 ]] || fail "zero-position requires an interactive terminal for safety confirmation"
        printf '%s\n' "=== Zero-position preview ==="
        "${ROBOT_ENV}/bin/python" "$ZERO_POSITION_SCRIPT"
        printf '\n%s\n' 'This moves the neck, both arms, and waist/leg joints.'
        printf '%s\n' 'Confirm navigation is stopped and the whole-body motion envelope is clear.'
        read -r -p 'Proceed with zero-position motion? [y/N] ' answer
        case "$answer" in
            y|Y)
                exec "${ROBOT_ENV}/bin/python" "$ZERO_POSITION_SCRIPT" \
                    --execute \
                    --safety-acknowledged
                ;;
            *)
                printf '%s\n' 'Zero-position canceled by operator; no command was published.'
                ;;
        esac
        ;;
    navigate)
        [[ $# -gt 0 ]] || fail "navigate requires an exact point name"
        exec "${ROBOT_ENV}/bin/python" "$NAVIGATE_SCRIPT" "$@"
        ;;
    *)
        usage >&2
        fail "unknown command: ${command_name}"
        ;;
esac
