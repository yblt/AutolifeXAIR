# AutolifeXAIR

XR 机器人挑战赛任务链（AutoLife S2）：导航、识别抓取、投放的示例源码与
一条龙入口。仓库目录布局与机器人部署目录
（`~/Documents/AutolifeXAIR`）一致：选手用 `rsync` 覆盖部署，工作人员
用 git 回退到发布基线。

现场排障先查 `TROUBLESHOOTING.md`（按症状分段的选手调试问题清单）。

## 开发机环境（仅跑测试/开发用）

```bash
./setup.sh        # 安装 pixi（如缺）并创建锁定环境
pixi run test     # 运行单元测试
```

机器人侧无需任何环境配置：运行时用机上预置的 miniconda `robot_env`
（Python 3.12）与厂商 SDK，请勿改动该环境。标定 JSON（根目录
`*.json`）是普通源码文件：发布基线携带一套可跑的出厂参考标定，
不改也能完成比赛；要调就在自己电脑上改、随代码一起部署。

## 部署（选手）

选手拿到的是不含 `.git` 的源码包，在自己电脑上改完后重新压缩，经 U 盘
交给工作人员；工作人员在 Ubuntu 电脑上解压到独立目录，再覆盖到机器人
（网线直连，机器人 `192.168.10.2`，用户名 `ubuntu`，密码由现场工作
人员提供）。选手若在 Windows 上解压/编辑过，脚本的可执行位必然丢失、
换行也可能变成 CRLF，所以上机前后各有一步固定修正，缺一不可：

```bash
cd <解压目录>/AutolifeXAIR
sed -i 's/\r$//' flow.sh setup.sh scripts/*.sh examples/navigation/nav-test.sh   # 去 CRLF
rsync -av --delete --exclude='.git' --exclude='evidence/' \
  ./ ubuntu@192.168.10.2:~/Documents/AutolifeXAIR/
ssh ubuntu@192.168.10.2 'cd ~/Documents/AutolifeXAIR \
  && chmod +x flow.sh setup.sh scripts/*.sh examples/navigation/nav-test.sh \
  && ./flow.sh'                                                           # 补执行位并预览自检
```

rsync 源路径的尾部斜杠不能省（表示同步目录内容而非目录本身）。
`--delete` 让机上与本地完全一致，本地删掉的文件机上也会删。没有
`rsync` 时用 `scp -r ./. ubuntu@192.168.10.2:~/Documents/AutolifeXAIR/`
（不会删除机上多余文件），之后仍要执行上面第三条的 `chmod +x`。
最后一句 `./flow.sh` 只做预览、不动机器人，输出正常即部署成功；
漏掉修正的症状见 `TROUBLESHOOTING.md` 0.7。

## 右臂链（备用）

仓库同时带两套抓取链，共用导航、复位与基础工具：

| | 左臂（默认） | 右臂 |
|---|---|---|
| 启用 | `./flow.sh` / `./flow.sh go` | `ARM=right ./flow.sh` / `ARM=right ./flow.sh go` |
| 跑器 | `examples/arm/*.py` | `examples/arm/right/*.py` |
| 标定 | 根目录四份 `*.json` | `right/*.json` |
| 手相机 | 左腕 `mod_camera_hand_left` | 右腕 `mod_camera_hand_right` |
| 验证 | autolife-robot-260 全链跑通（2026-08-22） | autolife-robot-274 全链跑通（切左臂前的主链） |

右臂链是切换到左臂之前那套代码与标定的快照副本，只修正了目录
引用，没有在 260 上复测；右腕相机在 260 上曾出现持续推流不稳，
这也是默认改为左臂的原因。要在一台新机器上启用右臂，先按
TROUBLESHOOTING.md 的相机检查确认右腕相机出帧，再按"调参速查"
改 `right/*.json`。两套跑器各自独立，改了一侧的逻辑不会自动
同步到另一侧。

## 工作人员

```bash
./scripts/deploy.sh     # 赛前发布基线（baseline 标签）
./scripts/rollback.sh   # 换队：机上恢复到基线，丢弃选手的一切改动（含 evidence/）
```

两个脚本网线直连机器人即可运行：有项目 SSH 密钥（`.codex/robot-ssh/`）
就免密，没有就提示输入一次 `ubuntu` 的密码，之后的 push/pull/校验复用
同一条连接。`deploy.sh` 还需要本地仓库配置机上裸仓库的 remote（新 clone
执行一次即可）：

```bash
git remote add robot ssh://ubuntu@192.168.10.2/home/ubuntu/Documents/AutolifeXAIR.git
```

`rollback.sh` 只恢复机器人，不动工作人员电脑上的仓库。选手的代码包请解压到
独立文件夹再 rsync 上机，不要覆盖进本仓库目录；若本仓库目录已被改乱，
用下面一条命令恢复成干净的 `main`（会删除目录内所有未跟踪文件）：

```bash
git checkout -- . && git clean -fdx
```

## 目录总览

```
AutolifeXAIR/
├── flow.sh                 # 一条龙入口：./flow.sh 预览、./flow.sh go 全链
├── bottle_grasp.json # 瓶抓标定配置 ─┐
├── basket_drop.json        # 瓶投标定配置   ├ 日常调参只改这四份
├── clothes_grasp.json      # 衣抓标定配置   │
├── clothes_drop.json       # 衣投标定配置 ─┘
├── right/                  # 右臂链的同名四份标定配置（ARM=right 时生效）
├── examples/arm/           # 四条任务链的状态机与配置守门模块（左臂）
├── examples/arm/right/     # 右臂版本的同名跑器与配置模块（快照副本）
├── examples/camera/        # 视觉检测引擎与几何反投影
├── scripts/                # 夹爪/复位/停稳门等基础工具
└── evidence/               # 运行时证据输出（定期归档清空）
```

导航侧入口也在本目录：`examples/navigation/nav-test.sh`（菜单、命名点导航、
定位），flow 直接调用它，无仓库外安装件。

## 入口用法

比赛全链（T1 复位 → T2 抓瓶 → T3 投瓶 → T4 抓衣 → T5 投衣 → 回 T1）：

```bash
cd ~/Documents/AutolifeXAIR
./flow.sh        # 只读预览：校验活动地图与 T1–T5 点位，零动作
./flow.sh go     # 实跑全链；键入 go 即现场唯一确认（清场、监护、急停在手）
```

导航交互菜单（定位、去点、停止、查地图；nav-test 无全局命令，
一律从仓库目录调用）：

```bash
cd ~/Documents/AutolifeXAIR
./examples/navigation/nav-test.sh menu
```

### flow 各段动作命令

`./flow.sh go` 按以下顺序跑段，任一段非零退出即原地中止（手臂保持当前状态）：

```
reset@T1 -> nav_T2 -> bottle_grasp -> reset_hold_bottle -> nav_T3 -> bottle_drop
-> reset_after_bottle -> nav_T4 -> clothes_grasp -> reset_hold_cloth -> nav_T5
-> clothes_drop -> reset_final -> nav_T1
```

各段等价的手跑命令（在 `~/Documents/AutolifeXAIR` 下）。复位包装与 nav-test
自带环境；直呼 python 的四个抓投段要先做环境前置（同下文"运行环境前置"，
`flow.sh go` 已代劳）。python 不需要 `conda activate`，直接用 robot_env 的
绝对路径即可：

```bash
# 前置：仅直呼 python 的段需要（CYCLONEDDS_URI 保持机上预置，不要改写）
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 ROBOT_ID="$(hostname | sed -e 's/.*-//')" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # 机器编号按主机名推导，勿写死
export PYTHONPATH="/home/ubuntu/Documents/AutolifeRobotSDK/sdk_python:$PYTHONPATH"
PY=/home/ubuntu/miniconda3/envs/robot_env/bin/python

# 复位段（reset@T1 / reset_hold_bottle / reset_after_bottle / reset_hold_cloth / reset_final）
./scripts/run_reset_robot.sh --execute --safety-acknowledged
# flow 在复位后额外等 15 s，再借 arm_move 1 mm 微动读左臂 EEF，
# 校验回位（0.12 < x < 0.18 且 z > 0.95），不满足即中止。

# 导航段（nav_T2 / nav_T3 / nav_T4 / nav_T5 / nav_T1，换点名即可）
./examples/navigation/nav-test.sh navigate T2 --execute --safety-acknowledged --feedback-timeout 180
$PY scripts/wait_base_settle.py --still-seconds 3 --timeout 60   # 等底盘停稳再动手臂

# bottle_grasp — T2 抓瓶
$PY examples/arm/run_bottle_grasp.py run --execute --reset-confirmed --config bottle_grasp.json

# bottle_drop — T3 投瓶
$PY examples/arm/run_basket_drop.py run --execute --config basket_drop.json

# clothes_grasp — T4 抓衣
$PY examples/arm/run_clothes_grasp.py run --execute --reset-confirmed --config clothes_grasp.json

# clothes_drop — T5 投衣
$PY examples/arm/run_clothes_drop.py run --execute --config clothes_drop.json
```

每段起止时间与退出码写入 `evidence/flow/<时间戳>/chain_log.csv`，段名与上面
一致；某段失败后用上表对应命令单独恢复/重试该段。

### 调参速查（在哪里改什么）

标定 JSON 在本目录根下，运行时以机上这四份为准，改完下次运行即生效
（run 模式在发出任何运动目标之前做范围校验，超限拒跑）；机上调好的值
要保留就手动拷回 PC 审阅提交，避免与仓库漂移：

- 抓瓶 `bottle_grasp.json`：
  - `hsv_lower` / `hsv_upper` — 瓶身色块 HSV（黄、蓝等用普通区间；红色
    这类跨 H=0 的颜色才写 lower H > upper H 环绕），配套 `min_area`、
    `detection_roi_px`
  - `y_step_m` — 伺服对中时左右每步距离（上限 0.005 m，累计上限见
    `bottle_grasp_config.Y_CUMULATIVE_MAX_M`）
  - `x_coarse_step_m` / `x_fine_step_m` — 远处/近处每步前进距离
    （上限 0.05 / 0.005 m）；`fine_zone_width_ratio` 是远近分界：色块
    宽度占比 ≥ 此值切细步
  - `final_forward_m` — 闭爪前最后一段固定前伸（上限 0.15 m）
  - `grasp_z_m` — 张爪后一次性下降到的绝对抓取高度（不得低于观察锚点
    下方 0.25 m，超了拒跑；观察锚点即复位后张爪时的实际位置）
- 抓衣 `clothes_grasp.json`：
  - `hsv_lower` / `hsv_upper` — 衣物色块 HSV，当前蓝色 `[100, 80, 40]` /
    `[130, 255, 255]`（OpenCV H 范围 0–179）
  - `min_area`（当前 3000）、`detection_roi_px`（当前 `[100, 100, 440, 240]`，
    含义 x/y/w/h）— 最小色块面积与头部相机检测 ROI
  - `erode_kernel_px`（当前 5）— 取抓取点深度前对衣物掩膜的腐蚀核，
    配套 `min_valid_depth_px` 有效深度像素下限
  - 颜色检测代码位置：`examples/camera/detect_color_targets.py`
    （`clothes_*` 参数卡 + 掩膜深度中值 `masked_depth_median`）委托
    `examples/camera/detect_bottle.py` 的 HSV inRange + 形态学 +
    连续帧稳定门引擎；代码内 `CLOTHES_DEFAULTS` 只是兜底默认，
    生效值一律以本 JSON 为准
- 投瓶/投衣同理改 `basket_drop.json` / `clothes_drop.json`。

换瓶子目标颜色（例：黄瓶 → 蓝瓶）只改 `bottle_grasp.json`，文件名与
flow 段名都不用动：

- `hsv_lower` / `hsv_upper`：红色是跨零环绕写法 `[170,80,50]` /
  `[10,255,255]`；改蓝色用普通区间，如 `[100,80,50]` / `[130,255,255]`
  （与抓衣的蓝色段一致，S/V 下限按现场光照微调）
- `target_id`：改成描述新目标的文字（自由文本，只写进证据记录，
  但要求如实，如 `blue-bottle-500ml`）。`camera_id` / `workcell_id`
  则不要改：runner 实跑前会与内置的相机/工位标识严格比对，不一致拒跑
- 瓶子只是颜色不同、尺寸相同时，其余参数不动；若瓶径也变了，才需要
  重标 `min_area`、`close_width_ratio_min/max` 与
  `close_position` / `grip_feedback_center`

改完先用"爪子相机当前画面"片段拍一帧确认新 HSV 能命中色块，再实跑。
注意只有抓衣 runner 的 `preview` 会加载并校验 JSON；抓瓶、投瓶、投衣三个
runner 的 `preview` 只打印计划与限值，配置错误要到 `run` 模式才报出
（仍在任何运动之前，fail-closed）。

固定动作与限幅常量在 `examples/arm/bottle_grasp_config.py` 顶部常量
区：抓瓶前的后退量 `BACK_CLEARANCE_M`（2026-08-21 起为 0，阶段保留但
零位移）、抓后抬升 `LIFT_M`（0.10 m）、下降下限
`GRASP_Z_RELATIVE_FLOOR_M`（0.25 m）以及上述 JSON 步长的上限。这些属于
安全限幅，调整前须经人工评审确认，不随日常调参改动。

### flow 加新动作段

链条只在 `flow.sh` 开头的 `chain` 数组里定义一次，在想要的位置插一行
即可，数组顺序就是执行顺序：

```bash
chain=(
  ...
  "task:new_action:examples/arm/run_new_action.py:new_action.json"   # 动作段
  "task:xxx_grasp:examples/arm/run_xxx.py:xxx.json:--reset-confirmed"  # 抓取段带额外 flag
  "nav:T6"                                                             # 导航段
  "reset:reset_after_new"                                              # 复位段
  ...
)
```

usage 里的链条文字、preview 的点位校验（只看 `nav` 条目）和必备文件
检查（只看 `task` 条目的脚本与 JSON）都从这个数组自动派生，不用另改。
新动作段的 runner 须遵守现有 runner 的命令约定（`run --execute --config
<json>`，默认 preview、失败非零退出）；flow preview 不会代跑新 runner
的配置校验，JSON 合法性以 runner `run` 模式运动前的校验为准（见
「调参速查」）。段名自动写入
`chain_log.csv`，任一段非零退出即整链中止。flow 在仓库根目录，
改完提交后经 git 同步到机器人。

### 导航：策略、到点容差与避障距离

策略链路：预建地图 + 命名点（厂商 GV 服务维护，`nav-test.sh` 经 `get_maps`
校验活动地图与点名）→ `nav-test.sh navigate <点>` 发 Nav2 NavigateToPose
（全向底盘，平移 0.4 m/s、旋转 1.0 rad/s）→ 到达判定双信号：主信号
`is_navigating` 连续 2 帧 false（含末端偏航对齐），备用为进度 JSON 推断
（门限在 `examples/navigation/navigate_to_named_point.py` 顶部常量区）
→ flow 再过 `wait_base_settle.py` 3 s 静止窗才动手臂。

参数在厂商栈（不在本目录）：
`~/miniconda3/envs/robot_env/lib/python3.12/site-packages/autolife_robot_gv/ros_ws/nav2_params.yaml`
（运行中的导航栈加载的是这份；同目录 `nav2_params_sf.yaml` 为未启用的
贴障参考版）：

- 到点容差：`xy_goal_tolerance: 0.05`（m）、`yaw_goal_tolerance: 0.05`
  （rad，≈2.9°；出厂 0.01/0.015 低于定位噪声会到点后蹭位不停）。改容差时
  须联动检查 nav-test 的 `ARRIVAL_DISTANCE_EPS_M`（0.15，须 > xy 容差 +
  0.05 一格量化）。
- 避障裕度：TEB `FollowPath.min_obstacle_dist: 0.20`（出厂 0.00，为零时
  贴墙角起步即 `trajectory is not feasible`）。
- 避障距离（膨胀层）：局部 `inflation_radius: 0.5` /
  `cost_scaling_factor: 3.0`；全局 `0.7` / `5.0`。半径越小越能过窄道、
  也越贴障碍。障碍感知量程 raytrace 局部 4 m / 全局 8 m。

改这份 YAML 需重启导航服务才生效；缩容差/缩膨胀属安全相关调整，改前
备份原文件并经人工确认，不随日常调参改动。厂商栈不随本目录部署，换机
或重刷镜像后以上值会回到出厂，按 `TROUBLESHOOTING.md` 附录 A 核对。

单步手控 / 失败后恢复：

```bash
./scripts/arm_move.sh --arm left --direction forward --distance 0.10            # 预览
./scripts/arm_move.sh --arm left --direction forward --distance 0.10 --execute  # 实动
./scripts/arm_move.sh --arm left --axis z --angle -2 --execute                  # 旋转
./scripts/arm_move.sh --arm both  --gripper open --execute                       # 夹爪
```

`arm_move.sh` 自动加载 ROS/conda/DDS/SDK 环境并展开底层安全门；平移单步
`<=0.30 m`、旋转 `<=20 deg`，方向以机器人自身视角为准（前 +X、左 +Y、上 +Z）。

## 各目录内容

### examples/arm/ — 任务链

| 文件 | 作用 |
| --- | --- |
| `run_bottle_grasp.py` | 抓瓶状态机（HSV 视觉伺服，左手相机） |
| `run_basket_drop.py` | 投瓶状态机（头部相机识别黄篓），也是投衣的共享引擎 |
| `run_clothes_grasp.py` | 抓衣状态机（复用投瓶引擎 + 衣物检测） |
| `run_clothes_drop.py` | 投衣状态机（复用投瓶引擎） |
| `*_config.py`（4 个） | 各链的配置加载与守门校验（相机/目标/工位 ID、限幅、必填字段） |
| `head_pitch.py` / `play_zero_position.py` | 头部俯仰 / 手臂回零 |

每个 runner 命令形式相同，默认 preview 零动作：

```bash
$PY examples/arm/run_bottle_grasp.py preview --config bottle_grasp.json
$PY examples/arm/run_bottle_grasp.py run --execute --reset-confirmed --config bottle_grasp.json
```

（`$PY` 是 robot_env 解释器，随"运行环境前置"一起设置，见下文；`flow.sh` 与
`arm_move.sh` 已代劳。不要用系统 `python3`：source ROS 后它有 rclpy 能动手臂，
但没有 posix_ipc，会在 SERVO 首次读手部相机帧时才 fail-closed 中止，见
`TROUBLESHOOTING.md` 2.1。）

### examples/camera/ — 视觉

| 文件 | 作用 |
| --- | --- |
| `detect_bottle.py` | 瓶身 HSV 检测引擎（含连续帧稳定门，衣物/黄篓检测复用它） |
| `detect_color_targets.py` | 衣物 + 黄篓检测（同一引擎的两组参数卡：`clothes_*` / `basket_*`） |
| `detect_head_bottle.py` | 头部相机瓶身检测 |
| `head_bottle_probe.py` / `head_bottle_geometry.py` | 头部 RGBD 采样探针 / 像素+深度→base 坐标反投影 |
| `read_hand_camera.py` | 手部相机 SHM 读帧 |
| `basket_drop_probe.py` | 投放定位只读探针（调参用，不发运动命令） |

### examples/navigation/ — 导航

| 文件 | 作用 |
| --- | --- |
| `nav-test.sh` + `nav_test_menu.py` | 导航菜单入口（定位、去点、停止、切图/查点位、整机复位） |
| `navigate_to_named_point.py` | 命名点导航（到达判定常量在文件顶部） |
| `capture_navigation_diagnostics.py` / `capture_split_lidar_overlay.py` | 定位对齐与雷达叠加的只读诊断 |

### scripts/ — 基础工具

| 文件 | 作用 |
| --- | --- |
| `run_reset_robot.sh` + `reset_robot.py` | 整机复位包装（`--execute --safety-acknowledged`） |
| `arm_move.sh` | 单步手控/恢复入口 |
| `control_gripper.py` | 夹爪开合底层 |
| `move_end_effector_relative.py` | 末端相对移动底层（读状态→完整双臂 payload→限幅→单发→等反馈，arm_move 调用） |
| `head_pitch.sh` / `head_probe.sh` | 头部俯仰 / 头部相机探针包装 |
| `wait_base_settle.py` | 导航到点后等底盘停稳的门（flow 各段间使用） |

## 深度引导抓瓶（低头 → 定位 → 抓取一条龙）

低头到 -30° → 头部深度相机定位瓶盖（含真值修正与爪尖工具偏移
0.228 m）→ 左臂悬停-进给-下探 → 按 `bottle_grasp.json` 的
`close_position` 合爪并校验 `grip_feedback_center ± close_tolerance`。
工具偏移 Y 分量为右臂值镜像、未独立复测，非比赛主链路。每段位移 ≤0.30 m 且逐段等停稳；任一阶段
失败原地停住，不重试、不自动松爪。

```bash
cd ~/Documents/AutolifeXAIR
./scripts/grasp_bottle.sh                                  # 预览：定位+航点计划，零动作
./scripts/grasp_bottle.sh --execute --safety-acknowledged  # 实跑（清场、监护、急停在手）
```

证据自动存 `evidence/depth_guided_grasp/<时间戳>/`（抓帧、检测标注、
定位与航点 JSON）。

## 低头与相机快照命令

低头/抬头（`--pitch` 是**绝对目标角不是相对增量**：先预览读当前角，再给
目标角。正 = 抬头、负 = 低头，2026-08-19 实物对照核验；允许带宽
[-40, +25] deg，单次调用变化 ≤20 deg，跨度大要分多步）：

```bash
cd ~/Documents/AutolifeXAIR
./scripts/head_pitch.sh                  # 预览当前脖子角度，零动作
./scripts/head_pitch.sh --pitch -15      # 干跑：校验并打印 payload，不发布
./scripts/head_pitch.sh --pitch -15 --execute --safety-acknowledged   # 实动
```

头部相机当前画面（color+depth 一对存 npz，另转一张 PNG；只读，
`--output-dir` 需是不存在的新目录，用时间戳避免重名）：

```bash
cd ~/Documents/AutolifeXAIR
d=evidence/camera-snapshots/$(date +%Y%m%d-%H%M%S)
./scripts/head_probe.sh capture --output-dir "$d" --frames 1
/home/ubuntu/miniconda3/envs/robot_env/bin/python - "$d" <<'EOF'
import sys, numpy as np, cv2
d = sys.argv[1]
ok = cv2.imwrite(f"{d}/frame_000_color.png", np.load(f"{d}/frame_000.npz")["color"])
print(("saved: " if ok else "FAILED: ") + f"{d}/frame_000_color.png")
EOF
```

头部深度相机（深度已随上面的 capture 一起存进同一 npz，单位毫米、
uint16、与彩色帧同 frame_id 配准；本片段渲染"近景带拉伸"伪彩图并读
画面中心距离：LO–HI 毫米区间映射蓝→红，超过 HI 涂灰、无效深度涂黑。
默认 550–1300 mm 适合低头看桌面的场景（约 2.9 mm/色阶，桌上物体与
桌面可分辨），看别的场景改 LO/HI 即可；全局 0.3–4 m 拉满会把局部差
异压平，桌面物体难以分辨，不要用）：

```bash
/home/ubuntu/miniconda3/envs/robot_env/bin/python - "$d" <<'EOF'
import sys, numpy as np, cv2
d = sys.argv[1]
LO, HI = 550.0, 1300.0   # mm, near-band for the lowered-head table view
depth = np.load(f"{d}/frame_000.npz")["depth"].astype(np.float32)  # mm
valid = depth > 0
h, w = depth.shape
win = depth[h//2-2:h//2+3, w//2-2:w//2+3]
win = win[win > 0]
print(f"depth valid: {valid.mean()*100:.1f}%  range: {depth[valid].min():.0f}-{depth[valid].max():.0f} mm")
print(f"center 5x5 distance: {win.mean():.0f} mm" if win.size else "center 5x5: no valid depth")
norm = ((np.clip(depth, LO, HI) - LO) / (HI - LO) * 255).astype(np.uint8)
vis = cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)
vis[~valid] = 0
vis[depth > HI] = (60, 60, 60)
ok = cv2.imwrite(f"{d}/frame_000_depth.png", vis)
print(("saved: " if ok else "FAILED: ") + f"{d}/frame_000_depth.png")
EOF
```

爪子相机当前画面（左手，比赛抓瓶链用的就是它；只读 SHM，无 ROS 依赖。
右手把 `left` 换成 `right` 即可。腕相机出流需机上视觉配置
`ENABLED_MODULES` 启用对应 `mod_camera_hand_left/right`，未启用或开机
时枚举晚于 vision-service 扫描则无 SHM 生产者、会报错退出，见
`TROUBLESHOOTING.md` 0.1）：

```bash
cd ~/Documents/AutolifeXAIR
mkdir -p "$d"   # 或复用上面头部快照的同一目录
/home/ubuntu/miniconda3/envs/robot_env/bin/python - "$d" <<'EOF'
import sys
sys.path.insert(0, "/home/ubuntu/Documents/AutolifeXAIR/examples/camera")
import read_hand_camera as rhc, cv2
out, side = sys.argv[1], "left"
gv, list_outputs, open_consumer = rhc._load_sdk()
settings = rhc._resolve_settings(rhc._parser().parse_args(["--side", side]), gv)
outputs = rhc._catalog_outputs(list_outputs, settings, rhc.SIDE_MODULES[side])
output = rhc._select_output(outputs, settings["output_name"], side)
consumer = rhc._open_consumer(open_consumer, output, side)
try:
    image, frame_id, _ = rhc._poll_frame(
        consumer, side, settings["timeout"], settings["poll_interval"])
    cv2.imwrite(f"{out}/hand_{side}.png", image)
    print(f"{side} frame_id={frame_id} -> {out}/hand_{side}.png")
finally:
    rhc._close_consumer(side, consumer)
EOF
```

## 运行环境前置（仅手动直呼 python 时需要）

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0 ROBOT_ID="$(hostname | sed -e 's/.*-//')" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # 机器编号按主机名推导，勿写死
export PYTHONPATH="/home/ubuntu/Documents/AutolifeRobotSDK/sdk_python:$PYTHONPATH"
PY=/home/ubuntu/miniconda3/envs/robot_env/bin/python
# python 一律用 $PY（robot_env）。系统 python3 在 source ROS 后有 rclpy
# 但缺 posix_ipc（SDK 相机 SHM 依赖），手臂会先动、SERVO 读帧才中止。
# CYCLONEDDS_URI 保持机上预置 loopback 配置，不要改写
```

## 安全约定（不可修改）

- 所有脚本默认 preview 零动作；实体动作必须显式 `--execute`，抓取链另需
  `--reset-confirmed`，复位/夹爪另需 `--safety-acknowledged`。
- 代码内运动限幅、双臂 payload 完整性检查、反馈收敛门、fail-closed 中止
  语义是安全底线；任何失败停在原地，不自动回退、不自动松爪、不自动重试。
- 需要立即停止时使用物理急停；软件中止不能取消控制器已接受的在途目标。
- 不要绕过包装直接向原生话题裸发位姿 JSON。

## 证据与归档

- 运行证据写入 `evidence/`（flow 每次全链在 `evidence/flow/<时间戳>/`
  记录分段耗时与退出码）。
- `evidence/` 不随部署同步（rsync 已排除），赛前由工作人员归档清空。
