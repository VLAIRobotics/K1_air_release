# K1 全向轮底盘 V1.2.6

此目录是 K1 全向轮底盘的当前上位机实现。它继承了 L1 V1.2.5 的白色遥控器动作 trigger、公共机械臂播放器与 systemd 安装方式，但**没有沿用差速轮遥测和速度换算**：K1 使用 `0x510`～`0x517` 八帧 V3 遥测、只接收的 ROS listener，以及独立的全向轮 SI 速度 CLI。L1 的 `/chassis/cmd` 和终端菜单由本目录的 K1 兼容控制器提供，不能直接运行旧版差速控制脚本。机械臂动作、底盘遥测和底盘速度发送是三个不同入口。

以下相对路径均从 SDK 根目录执行。Air 提供底盘控制和状态发布；数据集采集仅 Pro/Max 提供。

## 程序、CAN 与并行边界

| 程序 | 用途 | CAN 行为 |
|---|---|---|
| `chassis_system/chassis_telemetry_v3.py` | 解析并缓存 V3 八帧 | 不打开接口 |
| `chassis_system/chassis_listener.py` | 发布 ROS telemetry、odom、target、status | 只接收 `can2`，不发送 `0x050` |
| `chassis_system/chassis_omni_velocity_command.py` | 受限的 SI 速度命令 | 独立发送 `can2/0x050` |
| `chassis_system/chassis_control.py` | 兼容 `/chassis/cmd` 和终端菜单 | 独立发送 `can2/0x050`；与 SI CLI 二选一 |
| `chassis_system/can_listener_trigger.sh` | 白色遥控器机械臂动作触发 | 读取 `can2/0x051`，由公共播放器向机械臂 CAN 发动作 |
| `chassis_system/setup_autostart.sh` | 安装 CAN 配置及 trigger 服务 | 安装时会立即启用服务 |

触发器当前默认值按 2026-10-12 `vlai-Raymond` 实测接线设为底盘 `can2`、左臂 `can1`、右臂 `can0`；这**不是**自动识别，换机器仍须逐项核实。头部电机与右臂共用 `can0`，腰部电机与左臂共用 `can1`，但控制 ID 不同。动作播放器只控制各臂 `0x01`～`0x08`，按总线+电机组持有互斥锁；它不会独占整条 CAN。与头腰节点的同时运行仍需独立实机验收；未核实接线、其他发送进程及急停前，**不要启动 trigger、安装自启动或播放动作**。同一时刻只允许一个 trigger 和一个底盘 `0x050` sender；纯接收 listener 可与唯一的底盘发送程序并行。CAN 重新配置会中断已打开的 socket。

`xarm-can-listener.service` 名称容易误解：它运行的是白色遥控器机械臂 trigger，**不是** Python telemetry listener。做只读采集前先停止并确认状态：

```bash
sudo systemctl stop xarm-can-listener.service
systemctl is-active xarm-can-listener.service  # 预期 inactive
ip -details link show can2
```

## V3 八帧协议与 ROS

### 坐标方向与配套固件

配套的坐标修正版 STM32 固件统一使用：车体 X 向前、Y 向左、Yaw 从车顶俯视逆时针
（左转）为正。`0x510` 的位置使用与启动时车体坐标系对齐、之后不随车体转动的固定
坐标系；`0x512` 实际速度和 `0x516` 最终目标速度使用车体坐标系。Yaw 的原有正方向不变。
上位机解析、ROS 发布均直接使用下位机值，不额外取反，也不重新积分位置。

兼容控制接口 `0x050` 的横移 raw 仍为向右正，因此下面的 SI CLI 在发送边界对 Y 取反
一次。原遥控器混控和电机方向不变。该 CLI 须与坐标修正版固件配套部署；报文布局、
缩放和协议版本 `3` 未变化，程序不能据此自动区分旧的右正遥测。采集时记录固件提交及
坐标约定，新旧数据集分开保存；旧世界坐标位置不能靠最终 `y_m` 取负来修复。

STM32 正常以约 200 Hz、标准 Classic CAN、DLC=8、无 BRS、小端发送下列顺序。解析器拒绝扩展帧、错误帧和非 8 字节 DLC；当前代码没有单独以 SocketCAN `is_fd` 字段拒绝 FD 帧，因此不要把“协议要求 Classic CAN”写成“软件已完全强制”。

| ID | 小端字段 | 含义 |
|---|---|---|
| `0x510` | `int32 x_mm, y_mm` | 轮式里程计平面位置 |
| `0x511` | `int32 fused_yaw_mrad, uint32 timestamp_ms` | 连续融合航向与设备时间 |
| `0x512` | `int16 vx_mm_s, vy_mm_s, wz_mrad_s; uint8 slip, flags` | 实际车体速度 |
| `0x513` | `int16 RF, LF, LR, RR` | 四轮最终目标角速度，毫弧度/秒 |
| `0x514` | `uint16 seq; uint8 version, flags; uint32 drop` | V3 元数据 |
| `0x515` | `int32 wheel_yaw_mrad, mahony_yaw_mrad` | 两路航向诊断 |
| `0x516` | `int16 vx_mm_s, vy_mm_s, wz_mrad_s; uint16 reserved` | STM32 最终采用的目标车体速度 |
| `0x517` | `int16 RF, LF, LR, RR` | 四轮实际融合角速度；到达时触发快照 |

轮序固定 `[RF, LF, LR, RR]`。收到 `0x517` 时，解析器要求最近的 `0x510`～`0x516` 在 50 ms 窗口内且 `0x514.version == 3`，再形成 best-effort snapshot。各业务帧没有共同 seq，不能称为严格原子组。`0x514` 超过 100 ms 未到达时 status 标记离线；seq 按 uint16 回绕处理，drop 和协议错误保留为诊断计数。旧 `0x051` 只发布兼容 `/stm32/chassis_data`，不生成 K1 odom、target 或 Dataset。

| ROS 话题 | 类型 | 来源与条件 |
|---|---|---|
| `/chassis/telemetry_v3` | `std_msgs/msg/Float64MultiArray` | fresh V3 snapshot，字段顺序见 layout label |
| `/chassis/state` | `nav_msgs/msg/Odometry` | `0x510` x/y、`0x511` fused yaw、`0x512` 实际速度；须满足关键有效位 `0x13` |
| `/chassis/target_velocity` | `geometry_msgs/msg/TwistStamped` | `0x516` 最终物理目标速度 |
| `/chassis/telemetry_v3_status` | `std_msgs/msg/Float64MultiArray` | 约 5 Hz 的在线、seq/drop、slip/flags 与解析诊断 |
| `/stm32/chassis_data` | `std_msgs/msg/Float64MultiArray` | 仅旧 `0x051` 兼容数据 |

同一 snapshot 的 odom、target 与 host receive timestamp 均来自 `0x517` 的本机接收时间。fused yaw 为连续值；Odometry 四元数会自然按 `2π` 包裹。`valid_flags` 使用 `0x514`，slip 仅用于警告/显示，不在上位机重新平均三路 yaw。轮速误差定义为 target−actual。

只读 listener 用 ROS 已配置、装有 `python-can`/`rclpy` 的 Python 环境启动；先激活目标机对应环境，下例以已安装的 Humble 为例：

```bash
source /opt/ros/humble/setup.bash
python -B -u \
  publish/modules/src/xarm_chassis/V1.2.6/chassis_system/chassis_listener.py
```

启动后以 `ros2 topic hz /chassis/telemetry_v3`、`ros2 topic echo --once /chassis/state` 和 `/chassis/target_velocity` 核对数据；查看连续 yaw、各轮速度与 flags 时，以 telemetry layout label 为准，不仅依赖数组下标。PlotJuggler 可订阅这些 ROS 话题作趋势检查。

## 独立速度 CLI：`can2/0x050`

`--vx` 向前为正、`--vy` 向左为正，单位为 m/s；`--wz` 为 rad/s，逆时针为正。
CLI 按配套坐标修正版 STM32 的解码/逆解算换算为小端整数请求，不是直接把 SI 数值乘以 100：

```text
vx_raw = round_half_away_from_zero(+942.809041582 × vx_m_s)
vy_raw = round_half_away_from_zero(-942.809041582 × vy_m_s)
vw_raw = round_half_away_from_zero(-355.325871591 × wz_rad_s)
payload = <int16_le vx_raw, int16_le vy_raw, int16_le vw_raw, 0, 0>
```

byte7 在 STM32 中是 `updown` GPIO 字段，固定安全中性值 `0`，**不是 counter**。默认重发 50 Hz，允许 20～100 Hz；轴范围为 `|vx|,|vy|≤0.80 m/s`、`|wz|≤2.00 rad/s`，还会拒绝任一轮需求超过 `8 rad/s` 的组合。非零命令必须给正 `--duration`；结束、异常或 Ctrl+C 时至少持续 0.5 秒发送零帧。CLI 使用 `/tmp/xair-can2-0x050.lock` 防止自身双开，并检查已知 sender；STM32 AUTO 模式和 300 ms 命令超时仍须现场确认。

这些上限要求 STM32 已烧录四轮限幅为 `8 rad/s` 且遥测 Y 已修正为左正的配套固件，CLI
不会自动识别固件版本。换算倍率的绝对值和 CAN 各轴 `±800 raw` 硬限不变；请求向前
`0.10 m/s` 为 `+94 raw`，向左 `0.10 m/s` 为 `−94 raw`，向右为 `+94 raw`。
组合请求仍须满足四轮限幅，例如 `vx=0.40、vy=0.40 m/s` 允许，而
`vx=0.80、vy=0.80 m/s` 会因最大轮速需求约 `15.08 rad/s` 被拒绝。Python 脚本无需编译，
正常停止并重启 sender 后加载新转换；实车先低速核对 `0x516` 最终目标和实际反馈，再逐级提速。

仅计算 payload、不打开 SocketCAN 的检查：

```bash
python -B \
  publish/modules/src/xarm_chassis/V1.2.6/chassis_system/chassis_omni_velocity_command.py \
  --vx 0 --vy 0 --wz 0 --dry-run

# 左移请求：预期 vy_raw=-94，payload=00 00 a2 ff 00 00 00 00
python -B \
  publish/modules/src/xarm_chassis/V1.2.6/chassis_system/chassis_omni_velocity_command.py \
  --vy 0.10 --duration 1 --dry-run

# 右移请求：预期 vy_raw=+94，payload=00 00 5e 00 00 00 00 00
python -B \
  publish/modules/src/xarm_chassis/V1.2.6/chassis_system/chassis_omni_velocity_command.py \
  --vy -0.10 --duration 1 --dry-run
```

实车发送须另行授权，先被动抓包、核实唯一 sender/AUTO/急停，再做零帧和低幅度方向验收。下位机可能对直行施加航向保持修正，`0x516` 才是最终物理 target；不能用输入请求反推车辆一定移动了相同距离。不要与兼容控制器 `chassis_control.py`、旧版发送程序、其他 CLI 或 `cansend` 并行发送。

## 兼容 ROS 与终端控制

`chassis_control.py` 订阅 `/chassis/cmd`（`std_msgs/msg/String`），保留原 `1–5` 方向键和 `s/q`，`6/7` 为左右平移，立柱控制调整为 `8/9/0`，保持 30 Hz 重发。ROS 命令名称不变。它与现有 SI CLI 共用 CAN 发送和互斥锁，**不能同时运行**。V1.2.6 遥测 listener 仍然只读。

| ROS 命令 | 终端 | K1 行为 |
|---|---|---|
| `stop` | `1` | 清除底盘方向，不改变立柱指令 |
| `left` / `right` | `2` / `3` | 原地左转 / 右转，不是横移 |
| `forward` / `backward` | `4` / `5` | 前进 / 后退，使用 `vx_raw` |
| `strafe_left` / `strafe_right` | `6` / `7` | 左平移 / 右平移，分别发送负 / 正 `vy_raw`，前进和转向分量为零 |
| `lift_up` / `lift_down` / `lift_stop` | `8` / `9` / `0` | K1 的 byte 7 分别为 `1` / `2` / `0`；使用前核对实机固件 |
| `speed_0` ～ `speed_800` | `s` 后输入数值 | 设置 raw 速度，保留当前方向；默认 `306` |
| — | `q` 或 Ctrl+C | 发送底盘、立柱全零帧至少 0.5 秒，然后退出 |

方向和立柱命令会持续重发，直到收到新的命令或退出。`speed_0` 只把底盘速度设零，不清除方向，之后设置非零速度会按原方向恢复。raw 不是 m/s，也不经过 SI CLI 的输入限幅；启动后底盘和立柱均为停止。实机启动前须核对遥控器 AUTO、急停、CAN 配置与唯一发送者（包括旧 VR 发送进程）。

平移同样支持运行中发送 `speed_N` 改速，用 `stop` 停止；立柱状态不受影响。
兼容 CAN 横移指令仍为右正，故左移发负值，与 SI CLI 的发送方向一致。新增入口不修改固件或遥测，实机方向仍需低速验收。

在 Ubuntu 22.04／ROS 2 Humble 环境中，使用能导入 `rclpy` 的 Python。从 SDK 根目录执行；首次在该 Python 环境安装 `python-can` 后启动：

```bash
source /opt/ros/humble/setup.bash
python3 -m pip install python-can
python3 -B -u publish/modules/src/xarm_chassis/V1.2.6/chassis_system/chassis_control.py
```

另一个终端 source 同一 ROS 环境后，可发送兼容命令。以下示例用于刚启动、底盘和立柱均保持停止时设置待用速度：

```bash
timeout 5s ros2 topic pub --once /chassis/cmd std_msgs/msg/String '{data: stop}'
timeout 5s ros2 topic pub --once /chassis/cmd std_msgs/msg/String '{data: speed_120}'
```


## 白色遥控器机械臂动作与自启动

trigger 只解析 `can2/0x051` 的第 8 字节：SA（bit1）`1→0` 尝试动作；SD（bit6）`1→0` reset/unlock；SB（bit2/3）选套数；SC（bit4/5）选动作。启动后必须先松开 SD，再松开 SA。SD 与 SA 同帧松开时只解锁。动作中、结束后 1 秒冷却期、预留或非法组合均不重复触发，之后须再次解锁。

| SB | SC | 动作日志 | 当前默认目标接口 |
|---|---|---|---|
| `00` | `00` | `Rws.log` | `can0`（右臂） |
| `00` | `10` | `Rhs1.log` | `can0`（右臂） |
| `10` | `00` | `Rhs2.log` | `can0`（右臂） |
| `10` | `10` | `Lax.log`＋`Rax.log` | `can1`（左臂）＋`can0`（右臂） |

上述接口只对已记录的这台机器有参考价值，**不是**可执行许可。动作播放器和日志在公共 `xarm_action/action/`，版本目录不存二进制。

动作播放器要求参与臂的 7 个关节反馈 `0x011`～`0x017` 齐全；夹爪反馈 `0x018` 可选，缺失时提示并继续，旧夹爪指令仍正常发送。单臂动作不要求另一臂电机在线，但监听脚本启动仍要求左右臂 CAN 接口都存在且为 `UP`。有／无夹爪的兼容边界及重编译要求见[动作工具说明](../../publish/modules/src/xarm_action/action/README.md#有无夹爪电机的兼容行为)。

现场核实后，先在目标架构构建：

```bash
bash publish/modules/src/xarm_action/action/build_ramp_action_player.sh
```

再按现场审批手动运行 `bash publish/modules/src/xarm_chassis/V1.2.6/chassis_system/can_listener_trigger.sh`；正常退出用 Ctrl+C。脚本只清理由自身启动的播放器与 `candump`，不终止其他终端的进程。

`setup_autostart.sh` 以 root 运行，会覆盖系统唯一的 `xarm-can-setup.service` 和 `xarm-can-listener.service`、enable 并立即 start；unit 固定 `User=vlai`，trigger 路径取当前版本目录。**接线映射、服务账户、动作日志和急停均审核前不要运行安装器。**安装后 `xarm-can-listener.service` 仍仅代表动作 trigger；停止它不会停止 Python telemetry listener。
