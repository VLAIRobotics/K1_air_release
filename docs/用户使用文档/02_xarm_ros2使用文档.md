# K1 机械臂 ROS 2 统一启动与可视化控制

本文仅说明 K1 SDK 中公共机械臂模块，仍使用 xarm_description，不启动底盘、头腰或整机控制；产品专属功能按各自文档操作。CAN 接线必须按实际设备核对，不能跨产品照搬接口编号。

以下命令从 SDK 根目录开始。ROS 控制和主从遥操会访问硬件，不要让它们同时控制同一条机械臂。

## 首次编译

先按 SDK 安装说明完成 ROS Humble、rosdep 依赖及 yaml-cpp 0.8 安装。

```bash
source /opt/ros/humble/setup.bash
cd publish/modules
export XARM_SDK_ROOT="$(realpath ../xarm_can/package)"
colcon build --base-paths src/xarm_ros2 src/xarm_description \
  --cmake-args -DXARM_SDK_ROOT="$XARM_SDK_ROOT" -DBUILD_TESTING=OFF
source install/setup.bash
```

`BUILD_TESTING=OFF` 不构建开发测试。客户 Release 目前保留包内 `test/` 源文件，因为 CMake 引用了这些文件；正常启动不会执行测试。根目录内部 `tests/` 不进入客户 Release。

## 一个启动入口

编译后，从 `publish/modules` 执行：

```bash
bash src/xarm_ros2/start_xarm_ros2.sh --help
```

| 参数 | 可选值与说明 |
|---|---|
| `mode` | `single` 单臂；`bimanual` 双臂，默认 single |
| `scene` | `control` 基础控制；`moveit` 规划与 RViz，默认 control |
| `hardware` | `fake` 假硬件；`real` 真机，默认 real，建议始终显式填写 |
| `controller` | `trajectory` 时间轨迹；`position` 直接位置，默认 trajectory |
| `rviz` | 单臂基础控制是否显示 RViz，默认 false；MoveIt 自动开启 RViz |
| `can_interface` | 单臂 CAN，必须填写 |
| `single_side` | 单臂安装侧 left/right，必须填写，与 CAN 名称独立 |
| `right_can_interface` / `left_can_interface` | 双臂真机必须显式填写，且不能相同 |
| `--dry-run` | 只打印底层启动命令，不加载 ROS 或连接硬件 |

MoveIt 当前要求 `mode:=bimanual controller:=trajectory`。双臂基础控制不带 RViz；需要双臂可视化使用 MoveIt 场景。CAN、安装侧及白名单中的底层配置可按 `参数:=值` 传入；未知参数直接报错。底层配置白名单为 `arm_type`、`description_package`、`description_file`、`runtime_config_package`、`controllers_file`。不支持自定义命名空间。避免重复指定同一个参数。

### 双臂假硬件＋MoveIt

```bash
bash src/xarm_ros2/start_xarm_ros2.sh \
  mode:=bimanual scene:=moveit hardware:=fake controller:=trajectory
```

RViz 中打开 MotionPlanning，Planning Group 选择 `left_arm` 或 `right_arm`，开始状态选当前状态，拖动末端标记设置目标，先点击 Plan 检查轨迹，再点击 Execute。当前分别定义左右臂规划组，没有双臂联合规划组。假硬件用于软件验证，不代表真机验收。

### 双臂真机＋MoveIt

先配置好 CAN。以下示例仅适用于右受控臂接 can1、左受控臂接 can3；按实际接线修改。

```bash
bash src/xarm_ros2/start_xarm_ros2.sh \
  mode:=bimanual scene:=moveit hardware:=real controller:=trajectory \
  right_can_interface:=can1 left_can_interface:=can3
```

此入口已启动硬件、控制器、MoveIt 和 RViz，不要再重复启动 bringup。启动会接管硬件，确认工作空间和设备状态后执行。

### 单臂基础控制

```bash
bash src/xarm_ros2/start_xarm_ros2.sh \
  mode:=single scene:=control hardware:=fake controller:=trajectory \
  can_interface:=can0 single_side:=left rviz:=true
```

连接真机时改为 `hardware:=real`，并核对实际 CAN 和安装侧。

### 双臂直接位置控制

下列命令从 `publish/modules` 执行。每次另开命令或检查终端，先加载环境；如果设置了 `ROS_DOMAIN_ID`，所有终端必须一致：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

先启动假硬件位置控制：

```bash
bash src/xarm_ros2/start_xarm_ros2.sh \
  mode:=bimanual scene:=control hardware:=fake controller:=position
```

该场景启动双臂位置控制器，不启动 MoveIt 或双臂 RViz。另开已加载环境的终端检查：

```bash
ros2 control list_controllers
ros2 topic echo /joint_states --once
```

预期 `left_forward_position_controller`、`right_forward_position_controller`、左右夹爪控制器和 `joint_state_broadcaster` 为 `active`。位置模式不要求左右 `joint_trajectory_controller` 为 active。

| 受控臂 | 位置目标话题 | 数组顺序 |
|---|---|---|
| 左臂 | `/left_forward_position_controller/commands` | `xarm_left_joint1`～`xarm_left_joint7` |
| 右臂 | `/right_forward_position_controller/commands` | `xarm_right_joint1`～`xarm_right_joint7` |

消息类型为 `std_msgs/msg/Float64MultiArray`，每次提供完整的 **7 个弧度值**；不是角度，也不包含夹爪。1°约为0.0174533 rad。不要直接按 `/joint_states` 数组顺序复制目标，必须用 `name` 对应 `position` 后按上表重排。

以下绝对目标仅用于全新启动、7轴初始均为0的 **fake 会话**。左臂只改变第7关节到0.05 rad（约2.86°）：

```bash
ros2 topic pub --once /left_forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]}"
ros2 topic echo /joint_states --once
```

预期 `xarm_left_joint7` 接近0.05，其余左臂关节维持0；右臂保留原目标。对应的右臂示例同样只在该 fake 会话执行：

```bash
ros2 topic pub --once /right_forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]}"
```

预期 `xarm_right_joint7` 接近0.05，左臂目标不受影响。左右话题独立，连续执行两条命令不是严格同步的双臂轨迹。`--once` 只发布一次，控制器随后使用收到的位置目标；Ctrl+C 停止发布工具不等于撤销已发送目标。

### 双臂真机基础位置启动

先退出假硬件会话，配置好实际 CAN，并确认没有遥操或其他程序占用双臂。以下仅以右 can1、左 can3 为例，按实际接线修改：

```bash
bash src/xarm_ros2/start_xarm_ros2.sh \
  mode:=bimanual scene:=control hardware:=real controller:=position \
  right_can_interface:=can1 left_can_interface:=can3
```

启动会接管双臂。真机使用相同的位置话题，但目标必须依据当前实际反馈和工作空间确定，不能照搬上面的全零附近示例。每条消息是7轴绝对目标，不是相对增量。

直接位置控制不会生成 MoveIt 避障路径，也没有 `time_from_start`：目标收到后在后续控制周期转发到硬件，而不是保证经过指定秒数到达。机械臂实际运动速度还取决于驱动和机械响应；一次发送较大目标差值可能造成快速运动。需要按时间插值时，先退出当前场景，再使用 `controller:=trajectory`；需要 RViz 规划执行时使用前文 MoveIt 入口。两种控制器不能同时占用同一组位置接口。

### 单臂直接位置控制

单臂接口沿用统一入口，下面仍是假硬件示例：

```bash
bash src/xarm_ros2/start_xarm_ros2.sh \
  mode:=single scene:=control hardware:=fake controller:=position \
  can_interface:=can0 single_side:=left rviz:=true
```

启动后，另开已加载环境的终端向 `/forward_position_controller/commands` 发送 `Float64MultiArray`，顺序为 `xarm_joint1`～`xarm_joint7`，不带 `left_` 或 `right_`：

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05]}"
```

该示例仅适用于全新 fake 会话，预期 RViz 当前模型和 `xarm_joint7` 反馈跟随目标。真机改为 `hardware:=real`，并根据实际当前位姿准备目标。

只使用本文统一参数。位置式模式参数及旧的硬件、控制器和显示参数已移除；双臂真机必须显式指定左右 CAN。

## 控制器关系

| 控制器 | 用途 |
|---|---|
| `joint_state_broadcaster` | 将硬件反馈发布为 `/joint_states` |
| `left/right_joint_trajectory_controller` | 接收 FollowJointTrajectory，按时间插值输出位置目标，供 MoveIt 使用 |
| `left/right_forward_position_controller` | 直接转发每臂 7 维位置数组 |
| `left/right_gripper_controller` | 接收夹爪动作目标 |

位置目标经 xarm_hardware 转换为 CAN SDK 的 MIT 命令，使用配置的 Kp/Kd 和重力补偿。轨迹控制与直接位置控制占用相同位置接口，不能同时激活。现有速度 YAML 和硬件力矩路径不等于完整可用的速度/力矩启动方案，统一入口只提供已配置的位置和轨迹模式。

## 多套环境隔离

同一 ROS 域不要重复启动同名控制系统。启动与检查终端统一设置一个未占用的域，例如 `export ROS_DOMAIN_ID=42`，然后再运行启动或检查命令。域隔离不隔离 CAN；必须先停止访问相同电机的其他控制程序。MoveIt、TF 和控制节点使用同一份渲染后的双臂模型。

## 状态检查与故障定位

另开终端，在 `publish/modules` 下：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /joint_states --once
```

双臂轨迹模式预期左右轨迹控制器、左右夹爪控制器及状态广播器 active。

- `install/setup.bash` 不存在：先在本机完成编译。
- 硬件激活提示反馈无效或超限：核对反馈值、零点、限位、实际 CAN；不要通过放宽限位绕过检查。日志 joint 编号从 0 开始。
- 控制器已加载或存在重复 controller_manager：检查本机残留进程与 ROS 域内其他节点，避免重复启动。
- RViz 能看到模型不代表硬件激活或轨迹执行成功，应同时检查控制器状态。

## 与 UI 遥操数采的区别

UI 主从遥操使用 xarm_teleop 的 ROS2 入口，并发布 leader/follower 的关节及夹爪话题。这里的 bringup `/joint_states` 不能直接替代采集器 teleop 数据源。使用主从遥操采集时，在 UI 勾选启用 ROS2 节点，不要同时启动本入口占用相同设备。
