#!/usr/bin/env bash
# 一键比赛链条：T1 复位 -> T2 抓瓶 -> T3 投瓶 -> T4 抓衣
# -> T5 投衣 -> 回 T1。
# 链条只在下方 `chain` 数组中定义一次；usage 文本、preview 点位
# 校验和 go 模式执行全部由它派生。
# fail-closed：第一个非零退出的环节立即终止整条链；各环节 runner
# 内部的所有安全门保持完全生效。
set -uo pipefail

# 链条的唯一事实源。条目格式：kind:field[:field...]
#   reset:<segment_name>
#   nav:<point>
#   task:<segment_name>:<script relpath>:<config relpath>[:extra_flag]
chain=(
  "reset:reset@T1"
  "nav:T2"
  "task:bottle_grasp:examples/arm/run_bottle_grasp.py:bottle_grasp.json:--reset-confirmed"
  "reset:reset_hold_bottle"
  "nav:T3"
  "task:bottle_drop:examples/arm/run_basket_drop.py:basket_drop.json"
  "reset:reset_after_bottle"
  "nav:T4"
  "task:clothes_grasp:examples/arm/run_clothes_grasp.py:clothes_grasp.json:--reset-confirmed"
  "reset:reset_hold_cloth"
  "nav:T5"
  "task:clothes_drop:examples/arm/run_clothes_drop.py:clothes_drop.json"
  "reset:reset_final"
  "nav:T1"
)

chain_text() {
  local entry kind name _rest label out=""
  for entry in "${chain[@]}"; do
    IFS=: read -r kind name _rest <<<"$entry"
    case "$kind" in
      reset) if [[ "$name" == reset@* ]]; then label="$name"; else label="reset"; fi ;;
      nav)   label="nav $name" ;;
      task)  label="${name//_/ }" ;;
      *)     label="$name" ;;
    esac
    if [[ -z "$out" ]]; then out="$label"; else out+=" -> $label"; fi
  done
  printf '%s\n' "$out"
}

chain_points() {
  local entry kind name _rest seen=" "
  for entry in "${chain[@]}"; do
    IFS=: read -r kind name _rest <<<"$entry"
    [[ "$kind" == "nav" ]] || continue
    [[ "$seen" == *" $name "* ]] && continue
    seen+="$name "
    printf '%s\n' "$name"
  done
}

usage() {
  cat <<EOF
Usage:
  ./flow.sh        Print the no-action plan and validate map/points (read-only).
  ./flow.sh go     Run the full chain. Typing go is the single on-site
                confirmation for the whole chain: scene clear, operator
                monitoring, physical emergency stop in hand.

  ARM=right ./flow.sh [go]
                Use the right-arm chain instead of the default left arm:
                task scripts come from examples/arm/right/ and calibration
                from right/*.json (verified on autolife-robot-274).

Chain:
$(chain_text | fold -s -w 66 | sed 's/ *$//; s/^/  /')

Any segment failure aborts the chain in place (arm keeps its state; use
scripts/arm_move.sh / examples/navigation/nav-test.sh to recover manually). Per-segment timing is written to
evidence/flow/<stamp>/chain_log.csv.
EOF
}

fail() { echo "flow: $*" >&2; exit 2; }

# 选臂：默认左臂（autolife-robot-260 全链验证）；ARM=right 切到右臂链
# （autolife-robot-274 验证）。左臂路径与改前完全一致，右臂只在此处改写路径。
arm="${ARM:-left}"
case "$arm" in
  left|right) ;;
  *) fail "ARM must be 'left' or 'right' (got: '$arm')" ;;
esac

# 把 task 条目的脚本/配置路径按所选臂改写（左臂原样返回）。
arm_script() { if [[ "$arm" == "right" ]]; then printf '%s\n' "${1/examples\/arm\//examples/arm/right/}"; else printf '%s\n' "$1"; fi; }
arm_config() { if [[ "$arm" == "right" ]]; then printf 'right/%s\n' "$1"; else printf '%s\n' "$1"; fi; }

script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
project_root="$(cd -- "$(dirname -- "$script_path")" && pwd)"
python_bin="/home/ubuntu/miniconda3/envs/robot_env/bin/python"
nav_bin="${project_root}/examples/navigation/nav-test.sh"
reset_sh="${project_root}/scripts/run_reset_robot.sh"
arm_move_bin="${project_root}/scripts/arm_move.sh"

mode="preview"
case "${1:-}" in
  -h|--help) usage; exit 0 ;;
  "") ;;
  go) mode="go" ;;
  *) fail "unknown argument: $1" ;;
esac

required=("$python_bin" "$nav_bin" "$reset_sh" "$arm_move_bin")
for entry in "${chain[@]}"; do
  IFS=: read -r kind name script config _flag <<<"$entry"
  [[ "$kind" == "task" ]] || continue
  required+=("${project_root}/$(arm_script "$script")" "${project_root}/$(arm_config "$config")")
done
for f in "${required[@]}"; do
  [[ -e "$f" ]] || fail "required file is missing: $f"
done

if [[ "$mode" == "preview" ]]; then
  echo "flow preview: validating active map and named points (read-only)..."
  while read -r p; do
    if "$nav_bin" preview "$p" >/dev/null 2>&1; then
      echo "  point $p: ok"
    else
      echo "  point $p: FAILED (wrong active map or missing point)"; exit 5
    fi
  done < <(chain_points)
  usage
  echo "No action taken."
  exit 0
fi

# ---- go 模式 -------------------------------------------------------------
set +u  # ROS 的 setup 脚本会引用未定义变量
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
# 机器编号按主机名后缀推导（autolife-robot-<N>），不写死；厂商 CameraTransformer
# 用 ROBOT_ID 拼关节状态话题，写错会一直 "[KDL] No joints_pos yet"。
export ROBOT_ID="$(hostname | sed -e "s/.*-//")"   # 一律覆盖，不信环境里的值
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>255</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>}"
export PYTHONPATH="/home/ubuntu/Documents/AutolifeRobotSDK/sdk_python:${project_root}${PYTHONPATH:+:${PYTHONPATH}}"

stamp="$(date -u +%Y%m%dT%H%M%SZ)"
evidence_dir="${project_root}/evidence/flow/${stamp}"
mkdir -p "$evidence_dir"
log_csv="${evidence_dir}/chain_log.csv"
echo "segment,start_epoch,end_epoch,duration_s,exit_code" > "$log_csv"

run_segment() {
  local name="$1"; shift
  local start end code
  echo ""
  echo "[flow] ==== ${name} ===="
  start="$(date +%s)"
  "$@"
  code=$?
  end="$(date +%s)"
  echo "${name},${start},${end},$((end - start)),${code}" >> "$log_csv"
  if [[ $code -ne 0 ]]; then
    echo ""
    echo "[flow] CHAIN ABORTED at segment '${name}' (exit ${code})." >&2
    echo "[flow] Robot keeps its current state; recover with scripts/arm_move.sh / examples/navigation/nav-test.sh." >&2
    echo "[flow] timing log: ${log_csv}" >&2
    exit "$code"
  fi
  echo "[flow] ${name} ok ($((end - start)) s)"
}

do_reset() {
  "$reset_sh" --execute --safety-acknowledged || return $?
  sleep 15
  local pose px py pz
  pose="$("$arm_move_bin" --arm "$arm" --direction forward --distance 0.001 2>/dev/null \
    | grep "current_${arm}_eef_pose" | sed 's/.*"position":\[//;s/\].*//')"
  [[ -n "$pose" ]] || { echo "flow: could not read the ${arm} EEF pose after reset" >&2; return 3; }
  IFS=, read -r px py pz <<< "$pose"
  # 复位窗口数值沿用右臂复位位形的取值（0.12<x<0.18、z>0.95）；
  # 左臂复位位形尚未独立复测。
  "$python_bin" - "$px" "$pz" <<'PYEOF'
import sys
x, z = float(sys.argv[1]), float(sys.argv[2])
sys.exit(0 if 0.12 < x < 0.18 and z > 0.95 else 1)
PYEOF
  local ok=$?
  if [[ $ok -ne 0 ]]; then
    echo "flow: ${arm} arm did not return to the reset pose (x=${px} z=${pz})" >&2
    return 3
  fi
  echo "[flow] reset pose verified (x=${px} z=${pz})"
}

do_nav() {
  "$nav_bin" navigate "$1" --execute --safety-acknowledged --feedback-timeout 180 || return $?
  # Nav2 到达后仍会微调；手臂工作必须等基座静止。
  # 此处"不要"取消残余的目标跟踪：基座静止不能证明 Nav2 已完成
  # 最终偏航对齐（静止窗口可能恰好落在旋转的间歇里），2026-08-20
  # 取消曾两次截断 p3/p2 的朝向。在有基于 is_navigating 的完成
  # 检查之前，手臂环节期间的残余微调是接受的取舍。
  "$python_bin" "${project_root}/scripts/wait_base_settle.py" --still-seconds 3 --timeout 60
}

for entry in "${chain[@]}"; do
  IFS=: read -r kind name script config flag <<<"$entry"
  case "$kind" in
    reset) run_segment "$name" do_reset ;;
    nav)   run_segment "nav_${name}" do_nav "$name" ;;
    task)  run_segment "$name" "$python_bin" "${project_root}/$(arm_script "$script")" run --execute \
             ${flag:+"$flag"} --config "${project_root}/$(arm_config "$config")" ;;
    *)     fail "unknown chain entry kind: $entry" ;;
  esac
done

echo ""
echo "[flow] CHAIN COMPLETE."
echo "[flow] timing log: ${log_csv}"
column -s, -t < "$log_csv"
