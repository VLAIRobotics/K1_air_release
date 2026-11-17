# K1 Air SDK

本地发布包标识：`20261117_3da0846_r2`。正式版本号尚未指定。

## 使用入口

本包提供机械臂 CAN SDK、ROS 2 控制、遥操作、Web UI、底盘与头腰控制。Air 不包含多相机管理、任何 LeRobot 数据集采集和策略推理；底盘控制、状态读取和 ROS 状态发布保留。

所有相对路径以解压后的本目录为起点。

- 首次安装、整体编译与 UI 启动：[Web UI 使用说明](docs/用户使用文档/07_xarm_ui使用文档.md)。
- CAN SDK：[SDK 使用说明](docs/用户使用文档/01_xarm_can_SDK使用文档.md)。
- ROS 2 与 RViz：[ROS 2 使用说明](docs/用户使用文档/02_xarm_ros2使用文档.md)。
- 遥操作：[遥操作使用说明](docs/用户使用文档/03_xarm_teleop使用文档.md)。
- K1 底盘：以 [V1.2.6 模块说明](publish/modules/src/xarm_chassis/V1.2.6/README.md) 为准。
- K1 头腰：[头腰使用文档](docs/用户使用文档/06_K1头腰使用文档.md)。
- Python CAN 扩展：[安装说明](publish/xarm_can/python/README.md)，可用 `bash scripts/install_can_python.sh` 安装。

机械臂预编译库按 x86_64/aarch64 分目录提供；Python 预编译扩展对应 CPython 3.10。头腰 SDK 已统一在 `publish/xarm_can/head_waist` 下，提供 x86_64/aarch64 的 CPython 3.10 预编译扩展，无需编译驱动源码。请按实际设备核对 CAN 接线。

本次底盘 SI 指令修复要求配套坐标修正版固件，具体坐标约定见 V1.2.6 模块说明。

后续改动记录在 [CHANGELOG.md](CHANGELOG.md)。

## 使用文档顺序

- [01_xarm_can_SDK使用文档](docs/用户使用文档/01_xarm_can_SDK使用文档.md)
- [02_xarm_ros2使用文档](docs/用户使用文档/02_xarm_ros2使用文档.md)
- [03_xarm_teleop使用文档](docs/用户使用文档/03_xarm_teleop使用文档.md)
- [04_xarm_action使用文档](docs/用户使用文档/04_xarm_action使用文档.md)
- [05_xarm_chassis使用文档](docs/用户使用文档/05_xarm_chassis使用文档.md)
- [06_K1头腰使用文档](docs/用户使用文档/06_K1头腰使用文档.md)
- [07_xarm_ui使用文档](docs/用户使用文档/07_xarm_ui使用文档.md)
