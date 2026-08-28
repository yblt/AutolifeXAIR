#!/usr/bin/env bash
# 多物品比赛链条：支持3种饮料 + 3色衣物
# 用法：
#   ./flow_multi.sh        预览（零动作）
#   ./flow_multi.sh go     实跑全链
#
# 配置文件：items.conf（定义物品、JSON、导航点）
# 原始单物品 flow.sh 保持不变，可用于单段测试。

set -uo pipefail

script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
project_root="$(cd -- "$(dirname -- "$script_path")" && pwd)"
items_conf="${project_root}/items.conf"
python_bin="/home/ubuntu/miniconda3/envs/robot_env/bin/python"
nav_bin="${project_root}/examples/navigation/nav-test.sh"
reset_sh="${project_root}/scripts/run_reset_robot.sh"
arm_move_bin="${project_root}/scripts/arm_move.sh"

# 选臂：默认左臂；ARM=right 切到右臂链
arm="${ARM:-left}"
case "$arm" in
  left|right) ;;
  *) echo "flow: ARM must be 'left' or 'right' (got: '$arm')" >&2; exit 2 ;;
esac

arm_script() { if [[ "$arm" == "right" ]]; then printf '%s\n' "${1/examples\/arm\//examples/arm/right/}"; else printf '%s\n' "$1"; fi; }
arm_config() { if [[ "$arm" == "right" ]]; then printf 'right/%s\n' "$1"; else printf '%s\n' "$1"; fi; }

mode="preview"
case "${1:-}" in
  -h|--help)
    cat <<EOF
Usage:
  ./flow_multi.sh        预览配置和点位（零动作）
  ./flow_multi.sh go     实跑全链（3饮料+3色衣物）

配置文件：items.conf
ARM=right ./flow_multi.sh [go]  使用右臂链
EOF
    exit 0 ;;
  "") ;;
  go) mode="go" ;;
  *) echo "flow: unknown argument: $1" >&2; exit 2 ;;
esac

# 读取 items.conf，跳过注释和空行
declare -a items=()
while IFS= read -r line; do
  [[ -z "$line" || "$line" == \#* ]] && continue
  items+=("$line")
done < "$items_conf"

if [[ ${#items[@]} -eq 0 ]]; then
  echo "flow: no items defined in $items_conf" >&2; exit 2
fi

echo "flow: ${#items[@]} items loaded from items.conf"

# 预览模式：验证配置和点位
if [[ "$mode" == "preview" ]]; then
  echo ""
  echo "=== 预览模式（零动作）==="
  echo ""

  # 收集所有唯一的导航点
  declare -A seen_points
  for entry in "${items[@]}"; do
    IFS='|' read -r name grasp_json drop_json grasp_point drop_point flags <<<"$entry"
    seen_points["$grasp_point"]=1
    seen_points["$drop_point"]=1
  done

  # 验证点位
  echo "1. 验证导航点位..."
  for point in "${!seen_points[@]}"; do
    if "$nav_bin" preview "$point" >/dev/null 2>&1; then
      echo "   point $point: ok"
    else
      echo "   point $point: FAILED (wrong active map or missing point)"
      exit 5
    fi
  done

  # 验证配置文件
  echo ""
  echo "2. 验证配置文件..."
  for entry in "${items[@]}"; do
    IFS='|' read -r name grasp_json drop_json grasp_point drop_point flags <<<"$entry"
    grasp_path="${project_root}/$(arm_config "$grasp_json")"
    drop_path="${project_root}/$(arm_config "$drop_json")"
    grasp_script="${project_root}/$(arm_script "examples/arm/run_bottle_grasp.py")"
    drop_script="${project_root}/$(arm_script "examples/arm/run_basket_drop.py")"

    # 判断是瓶子还是衣服
    if [[ "$name" == bottle_* ]]; then
      grasp_script="${project_root}/$(arm_script "examples/arm/run_bottle_grasp.py")"
      drop_script="${project_root}/$(arm_script "examples/arm/run_basket_drop.py")"
    else
      grasp_script="${project_root}/$(arm_script "examples/arm/run_clothes_grasp.py")"
      drop_script="${project_root}/$(arm_script "examples/arm/run_clothes_drop.py")"
    fi

    status="ok"
    [[ ! -f "$grasp_path" ]] && { echo "   $name: MISSING $grasp_json"; status="fail"; }
    [[ ! -f "$drop_path" ]] && { echo "   $name: MISSING $drop_json"; status="fail"; }
    [[ ! -f "$grasp_script" ]] && { echo "   $name: MISSING $grasp_script"; status="fail"; }
    [[ ! -f "$drop_script" ]] && { echo "   $name: MISSING $drop_script"; status="fail"; }
    [[ "$status" == "ok" ]] && echo "   $name: ok (grasp=$grasp_json, drop=$drop_json)"
  done

  # 打印执行顺序
  echo ""
  echo "3. 执行顺序..."
  echo "   reset@T1"
  for entry in "${items[@]}"; do
    IFS='|' read -r name grasp_json drop_json grasp_point drop_point flags <<<"$entry"
    echo "   -> nav $grasp_point -> $name grasp -> reset -> nav $drop_point -> $name drop -> reset"
  done
  echo "   -> nav T1 (返回)"
  echo ""
  echo "预览完成。确认无误后运行: ./flow_multi.sh go"
  exit 0
fi

# ---- go 模式 -------------------------------------------------------------
echo ""
echo "=== 实跑模式 ==="
echo "确认：清场、有人监护、急停在手？(输入 go 继续)"
read -r confirm
[[ "$confirm" != "go" ]] && { echo "已取消。"; exit 0; }

set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROBOT_ID="$(hostname | sed -e "s/.*-//")"
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
  "$python_bin" "${project_root}/scripts/wait_base_settle.py" --still-seconds 3 --timeout 60
}

# 开始执行
echo ""
echo "[flow] Starting multi-item chain: ${#items[@]} items"
echo "[flow] timing log: ${log_csv}"

# 初始复位
run_segment "reset@T1" do_reset

# 逐个处理物品
item_num=0
for entry in "${items[@]}"; do
  item_num=$((item_num + 1))
  IFS='|' read -r name grasp_json drop_json grasp_point drop_point flags <<<"$entry"

  echo ""
  echo "[flow] === Item ${item_num}/${#items[@]}: ${name} ==="

  # 导航到抓取点
  run_segment "nav_${grasp_point}_${name}" do_nav "$grasp_point"

  # 抓取
  if [[ "$name" == bottle_* ]]; then
    run_segment "grasp_${name}" "$python_bin" "${project_root}/$(arm_script "examples/arm/run_bottle_grasp.py")" run --execute \
      ${flags:+"$flags"} --config "${project_root}/$(arm_config "$grasp_json")"
  else
    run_segment "grasp_${name}" "$python_bin" "${project_root}/$(arm_script "examples/arm/run_clothes_grasp.py")" run --execute \
      ${flags:+"$flags"} --config "${project_root}/$(arm_config "$grasp_json")"
  fi

  # 复位（持物）
  run_segment "reset_hold_${name}" do_reset

  # 导航到投放点
  run_segment "nav_${drop_point}_${name}" do_nav "$drop_point"

  # 投放
  if [[ "$name" == bottle_* ]]; then
    run_segment "drop_${name}" "$python_bin" "${project_root}/$(arm_script "examples/arm/run_basket_drop.py")" run --execute \
      --config "${project_root}/$(arm_config "$drop_json")"
  else
    run_segment "drop_${name}" "$python_bin" "${project_root}/$(arm_script "examples/arm/run_clothes_drop.py")" run --execute \
      --config "${project_root}/$(arm_config "$drop_json")"
  fi

  # 复位（空手）
  run_segment "reset_after_${name}" do_reset
done

# 返回出发点
run_segment "nav_T1" do_nav "T1"

echo ""
echo "[flow] CHAIN COMPLETE. ${#items[@]} items processed."
echo "[flow] timing log: ${log_csv}"
column -s, -t < "$log_csv"
