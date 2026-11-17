# K1 UI 使用文档

> 版本范围：Air 提供基础 CAN/遥操/监控 UI；多相机管理、相机预览操作和机械臂数据采集仅 Pro/Max。
> 预览重连按钮按实际模块能力显示，Air 不新增相机功能。

## 1. 简介

`xarm_ui` 是基于 Flask + Vue.js 的 XArm 机械臂 Web 控制面板，提供 CAN 接口管理、遥操作控制、相机管理、数据采集等功能，可在浏览器中统一操作和监控机械臂系统。

## 2. 目录约定与文件清单

本说明中的项目路径均相对于 **SDK 根目录**，即包含 `publish/` 和 `docs/` 的目录。
执行标注“从 SDK 根目录执行”的命令前，请在解压后的 SDK 根目录打开终端。

```text
publish/modules/src/xarm_ui/
├── app.py              # Web 后端
├── start_ui.sh         # 启动脚本
├── requirements.txt    # Python 依赖
├── static/             # 静态资源
└── templates/
    └── index-v1.html   # 前端页面
```

## 3. 环境要求

- Python 3.10+
- ROS2 Humble
- `can-utils`（`cansend`、`ip` 命令）

## 4. 快速开始

### 4.1 首次编译与加载运行环境

先激活与 ROS2 Humble 兼容的 Python 3.10 环境。从 **SDK 根目录**执行：

`rosdep install` 会读取 `src` 下各软件包的 `package.xml`，通过系统包管理器安装缺少的编译及运行依赖。`--ignore-src` 跳过源码中已提供的软件包，`-r` 遇到部分失败仍继续，`-y` 自动确认，`--rosdistro humble` 指定 ROS 2 Humble。首次安装或新增依赖时执行即可；完成后检查有无失败项，平时启动无需重复执行。

若下载慢，查看终端中的下载地址：`packages.ros.org` 下载慢时，需要调整 **ROS 2 软件源**，仅更换 Ubuntu 软件源不能解决该地址的下载速度问题。国内镜像可参考 [清华大学 TUNA ROS 2 镜像使用说明](https://mirrors.tuna.tsinghua.edu.cn/help/ros2/)。本 SDK 使用 Ubuntu 22.04 / ROS 2 Humble，配置时选择 **jammy**，架构按实际设备选择（x86_64 对应 amd64，ARM64 对应 arm64）；不要直接照抄其他 Ubuntu 版本的示例。按镜像说明更新现有 ROS 2 源，避免重复配置，再执行 `sudo apt update`，重新运行下面的依赖安装命令，已安装的软件包会跳过。

```bash
# 首次部署：安装遥操作预编译库所需的 yaml-cpp 0.8
bash ./scripts/install_yaml_cpp_08.sh
source /opt/ros/humble/setup.bash
cd ./publish/modules
export XARM_SDK_ROOT="$(cd ../xarm_can/package && pwd)"

# 首次部署：安装各 ROS2 包声明的依赖（需已安装并初始化 rosdep）
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# 编译当前版本提供的全部 ROS2 包
colcon build --cmake-args -DBUILD_TESTING=OFF

# 仅在编译成功后加载环境，并回到 SDK 根目录
source ./install/setup.bash
cd ../..
```

也可直接使用 `colcon build`，会包含默认开启的测试构建；上面的命令关闭测试构建。
编译目录必须是 `publish/modules`，构建范围包含其中可发现的全部包，
包括交付版本中存在的 `multi_realsense`。UI 等 Python 脚本无需编译；CAN SDK 使用交付包中的动态库。
编译完成时应无 `Failed` 或 `Aborted` 包；若失败，先解决构建错误，再继续。

`/opt/ros/humble/setup.bash` 是 ROS2 Humble 的标准安装位置；若采用其他安装位置，
请加载对应的环境脚本。不要将其他电脑的 build/install 目录当作已完成当前机器的安装。

遥操作预编译库依赖 `libyaml-cpp.so.0.8`，Ubuntu 22.04 的默认系统包不能代替此版本。
上述安装脚本从官方 0.8.0 固定提交构建共享库，安装到系统标准目录 `/usr/local`，
需要网络、git、cmake、C++ 编译器及安装权限；已安装时跳过。系统 0.7 库可以保留，不能将其软链接伪装成 0.8。

#### 已完成编译：每次新开终端加载环境

无需每次启动 UI 都重新编译。在 **SDK 根目录**执行：

```bash
source /opt/ros/humble/setup.bash
source ./publish/modules/install/setup.bash
```

加载环境、检查依赖和启动 UI 应在同一个终端完成。

### 4.2 安装并检查 UI 依赖

从 **SDK 根目录**执行：

```bash
python3 -m pip install -r ./publish/modules/src/xarm_ui/requirements.txt
python3 -c 'import flask, flask_socketio; print("UI 依赖正常")'
```

需要相机预览时，再检查图像处理依赖：

```bash
python3 -c 'import rclpy, sensor_msgs, numpy, cv2; print("相机预览依赖正常")'
```

ROS 图像依赖不包含在 UI 的 requirements.txt 中。若导入失败，请确认已安装相应
ROS2 图像组件、OpenCV，并使用与 ROS2 Humble 兼容的 Python 环境。

### 4.3 启动 UI

从 **SDK 根目录**执行：

```bash
cd ./publish/modules/src/xarm_ui
bash start_ui.sh
```

在运行 UI 的电脑浏览器打开 [http://localhost:5000](http://localhost:5000)。
从其他电脑访问时，使用 `http://运行UI的电脑IP:5000`。
保持启动终端运行，按 `Ctrl+C` 关闭 UI。

脚本会依次检查 Python、安装 Web 依赖、检查 ROS2 环境和监听端口。
出现“端口已占用”时，不要重复启动或直接杀进程。先查看：

```bash
ss -ltnp 'sport = :5000'
```

若是已启动的 UI，可以直接访问；也可以使用其他空闲端口：

```bash
XARM_UI_PORT=5001 bash start_ui.sh
```

此时打开 [http://localhost:5001](http://localhost:5001)。脚本检测与后端监听使用同一端口。

### 4.4 按需配置 CAN

仅查看相机画面时，可直接进入第 5.3 节，无需配置 CAN 或启动遥操作。
使用机械臂前，在 UI 点击“配置 CAN 接口”；也可从 **SDK 根目录**执行：

```bash
sudo bash ./publish/xarm_can/package/libexec/setup_can_interfaces.sh
```

该操作需要访问 CAN 硬件及相应权限。

## 5. 功能说明

### 5.1 CAN 接口管理

- **配置 CAN 接口**：以 CAN-FD 模式初始化 can0–can3（bitrate 1 Mbps，dbitrate 5 Mbps）
- **检查接口状态**：查看各接口 UP/DOWN 状态

### 5.2 遥操作控制

- 选择右臂、左臂或双臂。双臂模式分别填写左右主臂、从臂 CAN，共4个不同接口；单臂模式填写该侧主从两个不同接口。
- 按实际接线选择 CAN，不同运行臂不能共用接口。默认右侧为主 can0 / 从 can1，左侧为主 can2 / 从 can3，仅作为填写示例。
- 勾选“启用 ROS2 节点”可发布主从关节话题，供 ROS 监控与数据采集使用。
- 右上角模型显示收到的 ROS 关节反馈，可旋转和缩放查看。使用主从遥操时勾选“启用 ROS2 节点”；左右臂分别显示反馈状态。没有反馈时不代表设备当前姿态，模型面板仅用于显示，不发送控制指令。
- 点击启动后，分别查看左右运行状态及日志。双臂模式下已有一侧按相同配置运行时，仅启动另一侧；修改已运行侧的配置前须先停止该侧。
- 选择左臂或右臂后点击停止，只停止所选侧；选择双臂则停止两侧。此 UI 不接管其他终端或其他服务启动的遥操作进程。
- **零点设置**：对指定 CAN 接口上的电机（ID 001–008）发送零点校准指令，按设备校准流程操作。

### Air 功能范围

本版本不提供相机预览或数据集采集；这些功能仅在 Pro/Max 提供。

### 5.3 ROS 系统监控

- 刷新查看当前活跃的 ROS2 话题和节点

### 5.4 系统日志

实时显示各模块日志，颜色区分：
- 蓝色：标准输出
- 黄色：警告
- 红色：错误
- 紫色：状态信息

## 6. API 接口

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/system/info` | 系统信息 |
| GET | `/api/process/status` | 运行中的进程列表 |
| POST | `/api/teleop/start` | 启动遥操作 |
| POST | `/api/teleop/stop` | 停止遥操作 |
| GET | `/api/can/status` | CAN 接口状态 |
| POST | `/api/can/setup` | 配置 CAN 接口 |
| POST | `/api/can/setzero` | 设置电机零点 |
| GET | `/api/ros/topics` | ROS2 话题列表 |
| GET | `/api/ros/nodes` | ROS2 节点列表 |

## 7. 注意事项

### 7.1 端口占用

默认监听端口为 5000。使用 `XARM_UI_PORT=5001 bash start_ui.sh` 更换端口；不会自动终止已有进程。

### 7.2 路径解析

`app.py` 中的工作空间路径（`PROJECT_ROOT`）按 `src/xarm_ui` 相对当前文件的上两级目录自动推导，对应 `publish/modules/`，无需手动修改。


### 7.3 执行命令时的当前目录

第 4.1、4.2 节及 CAN 配置命令从 SDK 根目录执行。
第 4.3 节执行 `cd ./publish/modules/src/xarm_ui` 后，当前目录变为 UI 目录；
此后可直接执行 `bash start_ui.sh` 或 `XARM_UI_PORT=5001 bash start_ui.sh`。
需再次执行 SDK 根目录命令时，请在 SDK 根目录另开终端，避免在 UI 目录重复拼接路径。
