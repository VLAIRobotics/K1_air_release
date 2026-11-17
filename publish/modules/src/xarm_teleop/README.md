# xarm_teleop：机械臂遥操作

本模块用 Leader 的关节状态控制 Follower。源码入口为 `control/xarm_unilateral_control.cpp` 和 ROS2 入口 `control/xarm_uilateral_control_ros2.cpp`；公共 C ABI 位于 `include/xarm_teleop_sdk.h`，核心库按架构位于 `prebuilt/xarm_teleop/lib/`。`config/leader.yaml` 与 `config/follower.yaml` 保存控制参数。SDK 二次开发背景见 [C SDK 参考](../../../../docs/用户使用文档/01_xarm_can_SDK使用文档.md)。

完整的启动模式、SDK API 和排错说明保留在[遥操作详细手册](../../../../docs/用户使用文档/03_xarm_teleop使用文档.md)。

## 启动脚本与模式

从仓库根目录查看脚本帮助：

```bash
bash publish/modules/src/xarm_teleop/start_xarm_teleop.sh --help
```

实际调用格式为：

```text
bash publish/modules/src/xarm_teleop/start_xarm_teleop.sh \
  <unilateral|bilateral|gravity|unilateral_ros2> \
  <right_arm|left_arm> <现场核实的leader_can> <现场核实的follower_can> \
  [--home 七个逗号分隔的弧度值]
```

`unilateral` 为 Leader→Follower；`bilateral` 为双边力反馈；`gravity` 为仅 Leader 的重力补偿示教；`unilateral_ros2` 还发布 ROS2 关节状态。`--home` 仅对 `unilateral` 模式有效，应先审核七关节值和机械限位。脚本只运行、不编译；它要求 `/opt/ros/$ROS_DISTRO/setup.bash`、`publish/modules/install/setup.bash` 与 `publish/xarm_can/package/lib` 下可加载的 SDK 库。缺少产物时，按脚本打印的命令在 `publish/modules` 构建。

**运动安全：**脚本有历史默认接口（右 Leader `can0` / Follower `can2`，左 Leader `can1` / Follower `can3`），但默认值不是当前接线检测。2026-10-12 `vlai-Raymond` 记录 `can0` 为右臂＋头部、`can1` 为左臂＋腰部。未确认 Leader、Follower、头腰共用总线及其他运行进程前，不以省略接口参数的方式启动真实硬件。准备急停，先在安全姿态下验收方向、限位和停止。每次只开一个控制进程；Ctrl+C 后核对电机失能。

ROS2 版的 `/robot_go_home` 服务与关节状态话题只在相应节点运行时存在；回零会使电机运动，不属于只读验证。详细节点名称与参数以 `launch/teleop.launch.py` 为准。与 K1Max Recorder 配合时，应按 LeRobot README（数据集采集仅 Pro/Max） 检查本侧 leader/follower 话题、胸相机与可选本侧腕相机，不要求另一侧先启动。

## 二次开发接口

以本目录的 [公开头文件](include/xarm_teleop_sdk.h) 为签名和错误码准则，旧通用文档里的函数原型曾遗漏 `config_dir`，不能照抄。接口按职责分为：

| 职责 | C API |
|---|---|
| 创建会话 | `xarm_teleop_create_unilateral`、`xarm_teleop_create_bilateral`、`xarm_teleop_create_gravity_comp` |
| 生命周期 | `xarm_teleop_start`、`xarm_teleop_stop`、`xarm_teleop_wait`、`xarm_teleop_is_running`、`xarm_teleop_destroy` |
| 配置和数据 | `xarm_teleop_set_home_position`、`xarm_teleop_set_joint_state_callback`、`xarm_teleop_set_full_state_callback` |
| 主动归位与诊断 | `xarm_teleop_go_home`、`xarm_teleop_get_last_error`、`xarm_teleop_version` |

`create_*` 在当前头文件的合同中已初始化 CAN 并使能电机，**不是只读构造函数**。典型生命周期为 `create_* → 配置回调/审核 home → start → stop → wait → destroy`；`go_home` 会阻塞并驱动真实电机。返回非零时用 `xarm_teleop_get_last_error()` 诊断，不继续尝试运动。回调输入的关节为弧度、通常 7 轴，夹爪单列；具体数组长度、模式限制与线程语义以头文件为准。
