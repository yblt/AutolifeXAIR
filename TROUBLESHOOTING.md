# 选手调试问题清单（现场排障用）

- 来源：2026-08-13 至 2026-08-22 两台实机（274 / 260）调试期间的 40 条事故记录，筛掉纯代码缺陷与一次性事件后改写为选手视角。
- 用途：现场跑 `flow.sh` / 单段 runner / `nav-test.sh` 时，看到症状先查这里；每条给出"怎么确认"和"怎么处理"，不做根因叙述。
- 本文不复制标定数值，参数以仓库根四份标定 JSON 的机上现值为准，流程与调参位置见 `README.md`；每次运行的原始证据在机上 `evidence/<段>/<时间戳>/`。本文自带排障所需的全部核对项，不依赖仓库外文档。
- 状态含义：**已修** = 仓库代码已修且实机验证；**已绕过** = 问题仍在，但流程已不受影响；**现场处理** = 环境/硬件原因，每次现场按本文处理；**开放** = 尚未修复，需操作者注意。
- 机器人 ID 以主机名后缀为准（`autolife-robot-260` → `260`），下文话题名中的 `<id>` 按此替换。

## 0. 开机后必查（每次上电）

| # | 现象 | 原因 | 怎么确认 | 怎么处理 | 状态 |
|---|---|---|---|---|---|
| 0.1 | 抓瓶段手臂已经张爪、抬到抓取高度，然后报 `... is catalogued but its shared memory could not be opened` 退出；观感像"手部相机坏了" | 腕相机 USB 枚举比 `vision-service` 开机扫描晚 1–2 min，模块没注册；相机本身通常完好 | `journalctl --user -u vision-service -b --no-pager \| grep -E "Assigned /dev/video\|not found\|could not capture"`；`ls /dev/shm \| grep -E "hand_left\|rgbd_head"` | `systemctl --user restart vision-service`，等 ≥40 s，再用 `read_hand_camera.py --side left --output-name JPEG` 确认 frame_id 递增 | 现场处理 |
| 0.2 | 投篮/投衣段 LOCATE 一直失败或超时，头部相机没有画面；`ls /dev/video*` 比平时少 | 头部 RealSense 435i 偶发 USB 3.0 枚举失败（内核 `error -71`） | `lsusb \| grep 8086:0b3a` 无输出；`journalctl -k -b \| grep "usb 2-1"` 有 `not accepting address` | 整机重启（软件重启 vision-service 无效）；重启后再按 0.1 检查全部相机都已注册 | 现场处理（偶发硬件） |
| 0.3 | 右腕相机能出几帧，持续取流几十秒后断流；内核反复出现 USB disconnect | 右腕相机线缆/接头/供电边际不良 | `journalctl -k -b \| grep "3-6.4"` 看是否反复 disconnect | 当前比赛链已改用**左臂**；如需右臂，先重插线缆并做 ≥3 min 持续取流验证 | 开放（已切左臂绕过） |
| 0.4 | 导航段全部被拒：`EXIT_MAP_ERROR`、`validation rejected`、`known_points` 里没有比赛点位 | `gv-slam-service` 每次启动（含整机重启）都把活动地图回落为 `sz_office` | `./examples/navigation/nav-test.sh preview <点位>` 显示 active_map 不对 | 运行 `./examples/navigation/nav-test.sh`，启动提示切换时输入 `y`，或菜单 `4 (maps)` 手动切；切完**必须重新初始定位** | 现场处理（厂商闭源，无法根治） |
| 0.5 | `./examples/navigation/nav-test.sh locate` 发了初始位姿但地图/激光不对齐，定位像没生效 | 急停按下时底盘不发轮里程计和 `odom→base_link` TF，AMCL 静默丢弃 initial_pose | `ros2 topic hz /topic_gv_wheel_odom_0_<id>` 应约 62 Hz，急停按下时无数据 | 松开急停 → 确认地图正确 → 重新 `./examples/navigation/nav-test.sh locate` | 现场处理 |
| 0.6 | 导航方向整体反向、机器人朝操作者冲 | 机器人摆位按"外观车头"判断了前向，或雷达 TF 配置被换机还原 | 定位后看 rviz/对齐检查中激光与墙线是否镜像 | 导航前向 = 外观车头；定位时按此摆位；换机后核对厂商栈 `robot_v2_2.json`（定位方法见附录 A 第 3 项）的 `mod_lidar_front`/`mod_lidar_rear` x 与 yaw 是否互换（274 出厂即互换，260 未复发） | 已修（274/260 均验证） |
| 0.7 | 机上脚本 `Permission denied`；或 `bad interpreter` / `$'\r': command not found` | 代码包经 Windows 解压/编辑后丢了可执行位；`.sh` 被存成 CRLF | `ls -l flow.sh scripts/*.sh` 看 `x` 位；`grep -l $'\r' flow.sh scripts/*.sh` 看 CRLF | 按 README「部署（选手）」的固定步骤：rsync 前 `sed -i 's/\r$//'` 去 CRLF，上机后 `chmod +x`；每次拷代码包都要做 | 现场处理（每次部署必做） |

## 1. 导航段

| # | 现象 | 原因 | 怎么确认 | 怎么处理 | 状态 |
|---|---|---|---|---|---|
| 1.1 | 某段导航起步就原地转圈、后退，几轮后 `Goal failed` | 厂商栈 `nav2_params.yaml` 缺 costmap 膨胀层、TEB `min_obstacle_dist` 为 0、到点容差过小；**换机会把这些修复全部还原** | 按本文附录 A 逐项核对 `nav2_params.yaml` 四项，用 `ros2 param get` 看运行态，不要只看文件 | 按附录 A 改参（先备份），重启 `gv-slam-service`，然后重新切图 + 初始定位（0.4/0.5） | 已修（换机后必须重做） |
| 1.2 | 到点后底盘不停地微调蹭位，手臂段开始了底盘还在动 | 到点容差过严（旧值）或目标点贴障碍物（桌子）太近，TEB 避障项与目标项拉锯 | 看 global costmap 目标格代价；桌子是否被挪得比录点时更近 | 容差已放宽到 0.05/0.05；若仍蹭动，现场把桌子恢复原位，或重录点位外移 0.10–0.15 m | 已修 / 现场处理 |
| 1.3 | 导航明显已到点（机器人已停），但 `nav-test.sh`/`flow.sh` 等到 2–3 min 超时才往下走 | 旧版靠 `distance_remaining` 推断到点，Nav2 停车后该值冻结在阈值外 | 当前版本已改用 `/robot_navigation_0_<id>/is_navigating` true→false 判到点 | 确认机上脚本是仓库最新版（`git log -1`），旧副本不要用 | 已修 |
| 1.4 | 到点后朝向偏，下一段相机看不到目标（投篮对着窗、抓瓶对着墙） | 曾试过到点立即 cancel，会砍断 Nav2 终段转向 | 当前 `flow.sh` 不发 cancel，以 `is_navigating` 为准 | 不要自行在 flow 里加 cancel；复发则重导航一次 | 已修 |
| 1.5 | `flow.sh` 第一个导航段就 `EXIT_MAP_ERROR` 退出，点位名大小写不符 | 重录点位后名字变成小写 `p1–p5`（或相反），`flow.sh` 写死的名字不匹配 | `./examples/navigation/nav-test.sh preview` 看 `known_points` | 统一 `flow.sh` 里的点位名与地图一致 | 现场处理 |
| 1.6 | `nav-test.sh` 菜单 5（整机复位）报路径不存在 | 机上有仓库外的旧副本（`~/.local/lib/nav-test`）指向改名前路径 | `which nav-test` 应无输出；`ls ~/.local/bin/nav-test ~/.local/lib/nav-test` 应报不存在（仓库脚本不在 PATH 上） | 只用 `~/Documents/AutolifeXAIR/examples/navigation/nav-test.sh`；若上述 `~/.local` 路径仍存在，即为残留旧副本，删除后重试 | 已修 |

## 2. 抓瓶段（腕相机视觉伺服）

| # | 现象 | 原因 | 怎么确认 | 怎么处理 | 状态 |
|---|---|---|---|---|---|
| 2.1 | 手臂动作（张爪/抬高）做完了，才在 SERVO 读首帧处报 `posix_ipc is not installed` 或 `No module named 'examples'` 退出 | 用错 Python 解释器（系统 python3 缺 `posix_ipc`），或 PYTHONPATH 没带项目根 | 看 run_record 的 reason | 一律用 `flow.sh` 或 README 里的 `$PY`（`robot_env` 的 python）启动；不要裸 `python` | 已修（PREFLIGHT 未加相机试读门，仍是先动臂后失败） |
| 2.2 | SERVO 首帧 `no qualifying candidate`，手臂停住不碰撞 | 瓶子在画面里出 ROI（垂直偏高/偏低）、或 HSV 不匹配当前瓶子、或光照 | 用 `read_hand_camera.py` 截帧看瓶子位置与 `detection_roi_px` / `vertical_window_px` 的关系 | 按 README「调参速查」重采 HSV / 调 ROI / 调 `grasp_z_m`；**换臂或换瓶后这些都要重标，不能镜像** | 现场处理 |
| 2.3 | SERVO 末段宽度比一步跳过闭合带上限，中止 | 旁观者手/前臂（肤色偏红）入镜，与瓶标掩膜连通 | 证据目录 annotated.jpg 里掩膜 bbox 左界突然爆开 | 清场：人退出腕相机视野或穿长袖；再跑 | 现场处理 |
| 2.4 | 瓶子贴近后检测丢失、宽度比停在闭合带以下，持续粗步后中止 | 瓶子贴近失焦，饱和度跌破 HSV 下限，掩膜碎片化 | 证据帧为"贴脸糊"的大色块 | 放宽该瓶 HSV S/V 下限，或微降 `close_width_ratio_min` | 开放（绿瓶遗留，红瓶未复现） |
| 2.5 | FINAL_FORWARD / SERVO 中段报 `timed out waiting for EEF target convergence`，手臂实际已到位 | 厂商 IK 在向身体中线伸展构型下姿态解偏 1–2°，超过旧的 1° 旋转门；另一种是右臂（透传侧）被带动几 mm 触发了对侧门 | 机上实读 EEF 与目标差：位置 <1 mm、姿态 1–3° 即是此症 | 默认旋转门已放到 3°，右臂门放宽到 2 cm/3°；更新机上代码到最新 | 已修（IK 偏差本身未解） |
| 2.6 | CLOSE_GRIP 后 VERIFY_GRIP 超时，但肉眼看瓶子已夹住 | `grip_feedback_center` 还是别的瓶子/别的臂的值；闭爪行程超过等待窗；堵转点逐次浮动 ±1 | 持瓶状态 `ros2 topic echo` 夹爪 `position` 读实际堵转值 | 回填该瓶该臂的 `grip_feedback_center`（左臂约 190、右臂约 200，以实测为准）；等待窗已延至 20 s | 现场处理（换瓶必做） |
| 2.7 | 伺服侧向追偏很远后才对准 | `Y_CUMULATIVE_MAX_M` 为 0.20，是用户批准的现行限位 | — | 不是缺陷；摆位尽量让瓶子在腕相机画面中央 | 已定案 |

## 3. 投篮 / 投衣段（头部相机定位）

| # | 现象 | 原因 | 怎么确认 | 怎么处理 | 状态 |
|---|---|---|---|---|---|
| 3.1 | LOCATE `no qualifying candidate (N rejected)`，手臂不动、瓶仍在手 | 篮子被挪位，掩膜中心出 `detection_roi_px`；或画面里两只篮子一左一右、ROI 罩在缝上 | 跑只读探针（`examples/arm/basket_drop_probe.py`）或存帧看篮子中心与 ROI | 把篮子挪回停点正前方；ROI 已按现场散布放宽，不要再放宽到能同时罩两篮 | 现场处理 |
| 3.2 | LOCATE/CONFIRM 报 `invalid depth` 或深度出 `[0.45,1.6]` 窗 | 篮沿参考像素落到背景（反光地面的黄色倒影把掩膜顶边抬高）或深度洞 | 证据帧 bbox 顶边比正常高、沿像素深度 ≈2.4 m 或全零 | 移除背景里的黄色杂物；LOCATE 已加 3 次有界重试；频繁复发可把 `depth_window` 5→9 | 开放（瞬态，重试可过） |
| 3.3 | CONFIRM 后 `envelope` 拒绝（release target 超 `envelope_x_m` 上界） | 停点比录点时远了几 cm，或篮桌被挪 | run_record 里 target x 与包络上界差值 | 先重导航就位；仍复发则把篮桌/停点恢复原位。`basket_drop.json` envelope_x 上界已放到 0.7 | 现场处理 |
| 3.4 | MOVE_TO_HOVER 零收敛超时，感知正常 | 目标 y 太偏向身体对侧，超出单臂可达区（左臂向右越中线），厂商控制器静默拒绝、不报错 | hover y 比当天成功运行更偏对侧 | 把篮子向持物臂一侧（左臂则向机器人左侧）挪几 cm；或调整停点朝向 | 现场处理 |
| 3.5 | MOVE_TO_HOVER 某一小步发出后手臂不动，10 s 超时（间歇性） | 厂商控制器偶发丢弃单发目标消息 | issued_targets 正常但某步无位移 | EEF 目标已改 3 连发；确认机上代码最新 | 已修 |
| 3.6 | VERIFY_HOLDING 拒执行（feedback 不在持物带） | `held_feedback_center` 是别的瓶/别的臂的值；或抓衣带与投衣带不一致 | run_record reason 里给出 feedback 与带 | 回填机上 JSON；抓衣/投衣两个带的下沿必须对齐 | 现场处理（换瓶/换臂必做） |
| 3.7 | LOCATE 超时且终端刷 `[KDL] No joints_pos yet` | 终端里 `export ROBOT_ID=<旧机号>`，厂商库订阅了不存在的关节话题 | `echo $ROBOT_ID` 与 `hostname` 后缀不一致 | `unset ROBOT_ID`；当前代码已按 hostname 强制回写，确保机上是最新版 | 已修 |
| 3.8 | 投篮手臂穿过篮子停到远侧，瓶底压到篮后 | 感知→基座链路系统偏差 | 手工摆位实量 EEF 与定位点差 | 已用 `hover_offset_m` 吸收；**换相机/换头部姿态/换臂后必须重做一次真值比对再实跑** | 已修 |

## 4. 抓衣段

| # | 现象 | 原因 | 怎么确认 | 怎么处理 | 状态 |
|---|---|---|---|---|---|
| 4.1 | LOCATE 报 `invalid depth_raw`（带 .5 的中位数） | 偶数个掩膜像素时中位数不是整数 | — | 已改 `median_low` | 已修 |
| 4.2 | PINCH 超时 | 闭爪行程长超过等待窗；或只捏到极薄一褶，反馈贴近全闭 | 事后读夹爪反馈：接近全闭（右 ~360 / 左按实测）即捏空 | 等待窗已 20 s；现场把衣堆整理厚一点再捏；布料带不放宽 | 现场处理 |
| 4.3 | PINCH 偶尔"通过"但其实没捏住 | 瞬时进带判定，长等待窗下闭爪路过带区可能误判 | 投衣段 VERIFY_HOLDING 会拦 | 低优先；投衣前持物检查兜底 | 开放 |

## 5. 手臂通用

| # | 现象 | 原因 | 怎么确认 | 怎么处理 | 状态 |
|---|---|---|---|---|---|
| 5.1 | EEF 目标发布成功、手臂一动不动、无任何报错 | 厂商 daemon 静默丢弃自碰撞 / 关节跳变过大的目标，只写自己的 stdout | `journalctl --user -u arm-control-service -b` | 从复位姿态不要直接向中线横移：先前伸再侧移；脚本已会等待并提示 | 已绕过 |
| 5.2 | 换臂（右→左）后各段参数全不对 | 镜像右臂参数不成立：相机符号、视轴、夹爪反馈、IK 姿态偏差都不同 | — | 每个字段实测重标；首次运动前做真值比对（安全门，不可跳过） | 已定案 |

## 6. 其他

| # | 现象 | 处理 | 状态 |
|---|---|---|---|
| 6.1 | `data-logger-service` 反复自重启 | 不影响比赛主链路；需要数据记录时再查 | 开放 |
| 6.2 | `head_pitch.py --pitch -15` 头反而抬起 | `--pitch` 是绝对角不是相对角；负 = 低头 | 非缺陷 |

## 看到中止时的通用顺序

1. 先读终端打印的 `reason` 和证据目录（`evidence/<段>/<时间戳>/run_record.json` 与帧 jpg），不要立刻重跑。
2. 对照本文找症状；fail-closed 中止时手臂停在安全点、瓶子仍在手中，可以直接纠正环境后重跑该段。
3. 同一段同一症状连续两次失败，停下查参数（JSON）和环境（篮子/桌子/光照/旁观者），不要第三次盲跑。
4. 每个问题只允许一次计划内实跑 + 一次纠正性重跑；预算用完保留证据，换人/换思路。

## 附录 A：换机 / 厂商重刷镜像后的核对清单

厂商栈（`~/miniconda3/envs/robot_env` 下）不在 git 部署通道内，换机或重刷镜像后其中的历史修复全部回到出厂状态。首次实跑前逐项核对：

1. `~/miniconda3/envs/robot_env/lib/python3.12/site-packages/autolife_robot_gv/ros_ws/nav2_params.yaml` 四项（274/260 两代实车验证值）：
   - local_costmap `plugins: ["static_layer", "voxel_layer", "inflation_layer"]`（出厂仅 `["voxel_layer"]`）
   - global_costmap `plugins: ["static_layer", "obstacle_layer", "inflation_layer"]`（出厂缺 `inflation_layer`）
   - TEB `FollowPath.min_obstacle_dist: 0.20`（出厂 0.00）
   - `general_goal_checker.xy_goal_tolerance: 0.05` / `yaw_goal_tolerance: 0.05`（出厂 0.01/0.015）

   改前备份 `nav2_params.yaml.bak-<日期>-<事由>`；改后 `systemctl --user restart gv-slam-service`，再用运行态复核：
   `ros2 param get --no-daemon /controller_server FollowPath.min_obstacle_dist` 等四项，且 local costmap 要出现 inflated 代价格（修复前恒为 0）。查询导航栈需沿用其 `CYCLONEDDS_URI`（loopback）环境并加 `--no-daemon`。
2. 改了 `xy_goal_tolerance` 必须同步复核 `examples/navigation/navigate_to_named_point.py` 的 `ARRIVAL_DISTANCE_EPS_M`（应大于容差 + 栅格量化 0.05 + 余量）。
3. 雷达 TF：厂商栈配置 `robot_v2_2.json`（用 `find ~/miniconda3/envs/robot_env -path "*/configs/robot_v2_2.json"` 定位，同名文件不止一份，认含 `mod_lidar_front` 的那份）的 `mod_lidar_front`/`mod_lidar_rear` x 与 yaw 是否互换（症状见 0.6）。核对法：用前后向激光距离对照物理净空。
4. 活动地图回落 `sz_office`：重启 `gv-slam-service` 后重新切图 + `./examples/navigation/nav-test.sh locate`（见 0.4/0.5）。
5. `autolife_robot_vision/configs/robot_v2_2.json`（同上 find 结果中 vision 包下的那份）的 `ENABLED_MODULES` 是否含所用腕相机模块（左臂链需 `mod_camera_hand_left`，出厂未启用）。
6. 仓库根四份标定 JSON 的机上现值是否与仓库一致（`git status` / `git diff`），换机后回到出厂值或缺失的按 README「调参速查」重标。

验收基线：比赛图五个点位连跑全部 `Goal succeeded`，controller 日志无 `not feasible`、无恢复行为。
