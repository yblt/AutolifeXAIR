#!/usr/bin/env python3
"""nav-test 的精简交互式操作员菜单。

这里只暴露与比赛相关的动作：初始定位（locate）、带防护的导航
（go）、取消、地图查看/切换，以及整机复位。地图编辑、SLAM 模式
切换、状态转换和底层厂商动作（order / reset_stage / update_config）
留在完整厂商菜单里，通过 ``nav-test.sh vendor`` 进入；零位动作仍以
``nav-test.sh zero-position`` 提供。

物理运动保持与 ``nav-test.sh go`` / ``nav-test.sh zero-position`` 子命令
相同的门禁：先只读 preview，再在真实终端上交互式 y/N 确认，最后
经带防护的辅助脚本、带上两个显式执行门执行。
"""

import json
import os
import subprocess
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from autolife_robot_srvs.srv import SetString

NAV_TEST_LIB = os.path.dirname(os.path.abspath(__file__))
NAVIGATE_SCRIPT = os.path.join(NAV_TEST_LIB, "navigate_to_named_point.py")
ZERO_POSITION_SCRIPT = "/home/ubuntu/Documents/AutolifeXAIR/examples/arm/play_zero_position.py"
# AutolifeXAIR 已有的带防护整机复位辅助脚本；它有自己的 preview
# 模式和同样的 --execute/--safety-acknowledged 双重门禁。
RESET_SCRIPT = "/home/ubuntu/Documents/AutolifeXAIR/scripts/reset_robot.py"
GO_FEEDBACK_TIMEOUT = "120"

# 首选工作地图：启动时若激活的是别的地图，交互菜单会提供一键切换
# （厂商 slam 服务开机会重置激活地图，我们这侧无法持久修复）。
DEFAULT_MAP = "XR123"

LOCATE_CHECKLIST = """初始定位检查清单:
  1. 确认急停已松开（急停按下时底盘无里程计，初始定位会被丢弃）。
  2. 将机器人物理摆放到当前活动地图上的一个已知点位。
  3. 确认外观车头方向与该点位的 yaw 一致（2026-08-13 修复雷达 TF 后，
     导航前向 = 外观车头；在 s2-test 的 point1 上即车头朝向 point6）。
  4. 在下面的列表中选择该点位并确认，发布 initial_pose。
  5. 发布后确认 map/laser 对齐、map -> odom 稳定，再开始导航。"""


class NavTestMenu(Node):
    def __init__(self) -> None:
        domain_id = os.getenv("ROS_DOMAIN_ID", "0")
        robot_id = os.uname().nodename.rsplit("-", 1)[-1]  # 机器编号一律按主机名推导，不信环境变量
        node_id = f"{domain_id}_{robot_id}"
        super().__init__("nav_test_menu")
        self.initial_pose_pub = self.create_publisher(
            String, f"/robot_navigation_{node_id}/initial_pose", 10
        )
        self.cancel_pub = self.create_publisher(
            Empty, f"/robot_navigation_{node_id}/cancel", 10
        )
        self.map_cmd_client = self.create_client(
            SetString, f"/robot_map_{node_id}/map_command"
        )

    def map_command(self, payload: dict, timeout: float = 5.0):
        if not self.map_cmd_client.wait_for_service(timeout_sec=2.0):
            print("错误: 地图服务不可用 (map_command)")
            return None
        request = SetString.Request()
        request.data = json.dumps(payload, ensure_ascii=False)
        future = self.map_cmd_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        response = future.result()
        if response is None:
            print("错误: 地图服务调用超时")
            return None
        if not response.success:
            print(f"错误: 地图服务调用失败: {response.result}")
            return None
        try:
            return json.loads(response.result)
        except ValueError:
            return {}

    def get_maps(self):
        data = self.map_command({"cmd": "get_maps"})
        if not data:
            return "", {}
        return data.get("active_map", ""), data.get("maps", {})

    def active_waypoints(self):
        active, maps = self.get_maps()
        positions = [
            position.get("name", "?")
            for position in maps.get(active, {}).get("prepared_positions", [])
        ]
        return active, positions


def pick_from(names: list, prompt: str):
    for index, name in enumerate(names, start=1):
        print(f"  {index}. {name}")
    print("  0. 取消")
    try:
        choice = int(input(prompt).strip())
    except (ValueError, EOFError):
        print("无效输入。")
        return None
    if choice == 0:
        return None
    if 1 <= choice <= len(names):
        return names[choice - 1]
    print("无效编号。")
    return None


def confirm(prompt: str) -> bool:
    try:
        return input(prompt).strip().lower() == "y"
    except EOFError:
        return False


def action_locate(menu: NavTestMenu) -> None:
    print(f"\n{LOCATE_CHECKLIST}")
    active, positions = menu.active_waypoints()
    if not positions:
        print("当前地图没有可用点位。")
        return
    print(f"\n当前地图: {active or '未知'}")
    point = pick_from(positions, "选择机器人当前所在点位编号: ")
    if point is None:
        print("已取消，未发布 initial_pose。")
        return
    if not confirm(f"确认机器人已物理摆放在 '{point}' 且朝向一致? [y/N] "):
        print("已取消，未发布 initial_pose。")
        return
    message = String()
    message.data = point
    menu.initial_pose_pub.publish(message)
    rclpy.spin_once(menu, timeout_sec=0.2)
    print(f"已发布 initial_pose: {point}")
    print("请核对 map/laser 对齐后再开始导航。")


def action_go(menu: NavTestMenu) -> None:
    active, positions = menu.active_waypoints()
    if not positions:
        print("当前地图没有可用点位。")
        return
    print(f"\n当前地图: {active or '未知'}")
    point = pick_from(positions, "选择导航目标点位编号: ")
    if point is None:
        print("已取消，未发布任何运动指令。")
        return
    print("\n=== Navigation preview ===", flush=True)
    preview = subprocess.run(
        [sys.executable, NAVIGATE_SCRIPT, point, "--feedback-timeout", GO_FEEDBACK_TIMEOUT]
    )
    if preview.returncode != 0:
        print("preview 失败，已中止，未发布任何运动指令。")
        return
    print(f"\n目标: {point}")
    print("确认定位正确、路线畅通、急停就绪、有人现场监护。")
    if not sys.stdin.isatty():
        print("非交互终端，拒绝执行物理导航。")
        return
    if not confirm("确认执行物理导航? [y/N] "):
        print("导航已取消，未发布任何运动指令。")
        return
    subprocess.run(
        [
            sys.executable,
            NAVIGATE_SCRIPT,
            point,
            "--feedback-timeout",
            GO_FEEDBACK_TIMEOUT,
            "--execute",
            "--safety-acknowledged",
        ]
    )


def action_stop(menu: NavTestMenu) -> None:
    menu.cancel_pub.publish(Empty())
    rclpy.spin_once(menu, timeout_sec=0.2)
    print("已发布导航取消指令 (cancel)。")


def switch_map(menu: NavTestMenu, target: str) -> None:
    print(f"正在切换到 '{target}'...")
    result = menu.map_command({"cmd": "switch_map", "map_name": target}, timeout=15.0)
    if result is not None:
        print(f"已切换到: {target}。切图后请先重新初始定位 (locate) 再导航。")
    else:
        print("切换失败。")


def action_maps(menu: NavTestMenu) -> None:
    active, maps = menu.get_maps()
    if not maps:
        print("无法获取地图列表。")
        return
    names = sorted(maps)
    print(f"\n=== 全部地图 === (激活: {active or '未知'})")
    for index, name in enumerate(names, start=1):
        info = maps[name]
        marker = " ← 当前" if name == active else ""
        waypoints = [p.get("name", "?") for p in info.get("prepared_positions", [])]
        print(f"  {index}. {name}{marker}  {info.get('description', '') or '(无描述)'}")
        print(f"     路点({len(waypoints)}): {', '.join(waypoints) if waypoints else '(无)'}")
    print("  0. 返回   (建图/删图请用 ./examples/navigation/nav-test.sh vendor 菜单 8 管理)")
    try:
        choice = int(input("输入编号切换地图 (0=返回): ").strip())
    except (ValueError, EOFError):
        print("无效输入。")
        return
    if choice == 0:
        return
    if not 1 <= choice <= len(names):
        print("无效编号。")
        return
    target = names[choice - 1]
    if target == active:
        print("已在该地图上。")
        return
    switch_map(menu, target)


def offer_default_map_switch(menu: NavTestMenu) -> None:
    active, maps = menu.get_maps()
    if not maps or active == DEFAULT_MAP or DEFAULT_MAP not in maps:
        return
    print(f"\n当前活动地图为 '{active or '未知'}'，默认工作地图为 '{DEFAULT_MAP}'。")
    if not confirm(f"是否切换到 '{DEFAULT_MAP}'? [y/N] "):
        print("保持当前地图不变。")
        return
    switch_map(menu, DEFAULT_MAP)


def action_reset(menu: NavTestMenu) -> None:
    print("\n=== Whole-robot reset preview ===", flush=True)
    preview = subprocess.run([sys.executable, RESET_SCRIPT])
    if preview.returncode != 0:
        print("preview 失败，已中止，未发布任何运动指令。")
        return
    print("\n整机复位会移动全身关节（双臂平放姿态）。")
    print("确认导航已停止、全身运动空间已清空、急停就绪、有人现场观察到动作完成。")
    if not sys.stdin.isatty():
        print("非交互终端，拒绝执行整机复位。")
        return
    if not confirm("确认执行整机复位? [y/N] "):
        print("整机复位已取消，未发布任何运动指令。")
        return
    subprocess.run(
        [sys.executable, RESET_SCRIPT, "--execute", "--safety-acknowledged"]
    )


MENU_ITEMS = [
    ("1", "locate        初始定位（选择点位发布 initial_pose）", action_locate),
    ("2", "go            安全导航到点（preview + y/N 确认）", action_go),
    ("3", "stop          取消当前导航", action_stop),
    ("4", "maps          查看/切换地图（全部）", action_maps),
    ("5", "reset         整机复位·双臂平放（preview + y/N 确认）", action_reset),
]


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    once = argv[0] if argv else None
    if once is not None and once != "locate":
        print(f"nav_test_menu: 未知参数: {once}", file=sys.stderr)
        return 2
    rclpy.init()
    menu = NavTestMenu()
    try:
        if once == "locate":
            action_locate(menu)
            return 0
        offer_default_map_switch(menu)
        actions = {key: handler for key, _, handler in MENU_ITEMS}
        while True:
            active, _ = menu.get_maps()
            print(f"\n=== nav-test 菜单 === [当前地图: {active or '未知'}]")
            for key, label, _ in MENU_ITEMS:
                print(f"  {key}. {label}")
            print("  0. 退出   (完整厂商菜单: ./examples/navigation/nav-test.sh vendor)")
            try:
                choice = input("选择操作: ").strip()
            except EOFError:
                print()
                return 0
            if choice == "0":
                return 0
            handler = actions.get(choice)
            if handler is None:
                print("无效选择。")
                continue
            handler(menu)
    except KeyboardInterrupt:
        print("\n已退出。")
        return 0
    finally:
        menu.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
