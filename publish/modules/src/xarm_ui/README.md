# xarm_ui

> 版本范围：Air 提供基础 CAN/遥操/监控 UI；多相机管理、相机预览操作和机械臂数据采集仅 Pro/Max。
> 预览重连按钮按实际模块能力显示，Air 不新增相机功能。

基于 Flask + Vue.js 的 XArm 机械臂 Web 控制面板，提供 CAN 接口管理、遥操作控制、相机管理、数据采集等功能。

## 环境要求

- Python 3.10+
- ROS2 Humble
- `can-utils`（`cansend`、`ip` 命令）
- Intel RealSense SDK（可选，用于相机功能）

## 安装与启动

以下项目路径均相对于 **SDK 根目录**（包含 `publish/` 和 `docs/` 的目录）。
首次使用请先完成整体编译。从 SDK 根目录执行（ROS2、colcon 和 rosdep 需已安装，rosdep 需已初始化）：

```bash
# 首次部署：安装遥操作预编译库所需的 yaml-cpp 0.8
bash ./scripts/install_yaml_cpp_08.sh
source /opt/ros/humble/setup.bash
cd ./publish/modules
export XARM_SDK_ROOT="$(cd ../xarm_can/package && pwd)"
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --cmake-args -DBUILD_TESTING=OFF
source ./install/setup.bash
cd ../..
```

也可用 `colcon build` 编译，包括默认测试构建。确认无失败包后继续。
以后启动时无需重复编译，先激活已安装 SDK 依赖的 Python 环境，再在 SDK 根目录执行：

```bash
source /opt/ros/humble/setup.bash
source ./publish/modules/install/setup.bash
python3 -m pip install -r ./publish/modules/src/xarm_ui/requirements.txt
python3 -c 'import flask, flask_socketio; print("UI 依赖正常")'
cd ./publish/modules/src/xarm_ui
bash start_ui.sh
```

ROS2 若安装在其他位置，请相应调整 ROS2 环境脚本位置。
Pro/Max 使用相机前应执行 `ros2 pkg prefix multi_realsense`。若提示 `Package not found`，
请先按完整使用文档第 4.1 节完成整体编译并加载环境；Air 无需执行相机包检查。Python 依赖检查通过不能代替 ROS2 包检查。
启动后在浏览器访问 [http://localhost:5000](http://localhost:5000)。
从其他电脑访问时，使用运行 UI 的电脑 IP。

端口被占用时，在 UI 目录执行 `XARM_UI_PORT=5001 bash start_ui.sh`，
并访问 [http://localhost:5001](http://localhost:5001)。按 `Ctrl+C` 关闭 UI。

使用相机预览前，在已加载运行环境的终端检查：

```bash
python3 -c 'import rclpy, sensor_msgs, numpy, cv2; print("相机预览依赖正常")'
```

仅查看相机画面无需配置 CAN。使用机械臂前，可点击“配置 CAN 接口”，
或从 **SDK 根目录**执行：

```bash
sudo bash ./publish/xarm_can/package/libexec/setup_can_interfaces.sh
```

## 功能说明

### CAN 接口管理

- **配置 CAN 接口**：以 CAN-FD 模式初始化 can0–can3（bitrate 1 Mbps，dbitrate 5 Mbps）
- **检查接口状态**：查看各接口 UP/DOWN 状态

### 遥操作控制

- 选择机械臂（右臂 / 左臂）
- 选择主臂和从臂 CAN 接口（从已检测接口中选择）
- **零点设置**：对指定 CAN 接口上的所有电机（ID 001–008）发送零点校准指令
- 启动 / 停止遥操作进程

### 相机管理

- 检测已连接的 Intel RealSense 相机
- 在“数据采集”页面检查相机，按胸部 D435、左腕 / 右腕 D405 分配序列号并启动
- 自动显示 RGB 预览；刷新后点击“开始 / 重连预览”，无需开始录制
- “关闭画面”仅关闭当前浏览器画面；“停止相机”停止相机节点

### 数据采集

- 配置数据集名称、任务描述、采集帧率等参数
- 启动 / 停止录制
- 支持 episode 控制（保存 / 重录）
- 使用对应产品采集模块要求的 LeRobot 版本；以该模块依赖与已验证环境为准

### ROS 系统监控

- 刷新查看当前活跃的 ROS2 话题和节点

### 系统日志

实时显示各模块日志，颜色区分：
- 蓝色：标准输出
- 黄色：警告
- 红色：错误
- 紫色：状态信息

## API 接口

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/system/info` | 系统信息 |
| GET | `/api/process/status` | 运行中的进程列表 |
| POST | `/api/teleop/start` | 启动遥操作 |
| POST | `/api/teleop/stop` | 停止遥操作 |
| GET | `/api/can/status` | CAN 接口状态 |
| POST | `/api/can/setup` | 配置 CAN 接口 |
| POST | `/api/can/setzero` | 设置电机零点 |
| GET | `/api/camera/check` | 检测相机 |
| POST | `/api/camera/launch` | 启动相机节点 |
| POST | `/api/camera/stop` | 停止相机节点 |
| POST | `/api/record/start` | 开始录制 |
| POST | `/api/record/stop` | 停止录制 |
| POST | `/api/record/control` | 控制 episode |
| GET | `/api/ros/topics` | ROS2 话题列表 |
| GET | `/api/ros/nodes` | ROS2 节点列表 |

## 目录结构

```
xarm_ui/
├── app.py              # Flask 后端
├── start_ui.sh         # 启动脚本
├── requirements.txt    # Python 依赖
├── static/             # 静态资源（JS、图片）
└── templates/
    └── index-v1.html   # Vue.js 前端页面
```


## 完整操作说明

启动、相机配置、实时预览和无画面排查步骤见
[《XArm UI 使用文档》](../../../../docs/用户使用文档/07_xarm_ui使用文档.md)。

## K1 Pro/Max 单臂 8D 采集

Web 录制按钮调用 LeRobot 模块（数据集采集仅 Pro/Max）的 `publish/lerobot_collector/xarm_ros2_record.py`，左右臂各自采集 7 关节＋1 夹爪。Recorder 只订阅 ROS，不打开 CAN、使能电机或发送运动命令；Web 也不会代替你启动 leader/follower 话题。实机采集前应先确认本侧四个机械臂话题、胸部相机话题均在持续发布；勾选腕相机时还需本侧腕部图像话题。

在“数据采集”页选 `left` 或 `right`，设置 repo-id、任务、FPS（1～120）以及是否使用本侧腕相机。保存路径可以留空：后端会在 `XARM_UI_DATASET_BASE`（默认 `~/lerobot_datasets`）下生成不会覆盖旧数据的新目录，并在日志中显示；手工填写时必须是**不存在的绝对路径**。实际保存目录由运行环境的 HOME 或 XARM_UI_DATASET_BASE 决定；在当前 work 容器中需要与主机共享时请选择 /root/work 下的新目录。

Web 后端默认优先使用 `/opt/miniconda/envs/lerobot/bin/python3` 运行 Recorder，否则使用 Web 自身的 Python。其他环境可在启动 Web 前设置 `XARM_UI_RECORDER_PYTHON=/实际路径/python`；ROS 环境脚本可通过 `ROS_SETUP` 指定，默认 `/opt/ros/humble/setup.bash`。需要 Python 3.10、ROS Humble 和兼容的 LeRobot 环境；只看见“录制进程已启动”不代表已采到帧，应等待 `Recorder ready`、`Saved K1Max episode` 日志并重载数据集核对 8D 向量与视频。

Episode 按键与当前 Recorder 一致：`r` 开始/暂停，`s` 保存，`d` 丢弃未保存内容；“停止录制进程”发送 `q`，会丢弃未保存的 buffer 并正常结束。Web 不提供旧版 `h/n` 回初始位置功能，不会因保存而驱动机械臂。页面或 Web 后端重启后若丢失进程句柄，不能依赖 Web 按钮停止先前的 Recorder，须到终端检查该进程并安全结束。**Web 实机录制与视频保存尚待现场验收**；离线接口测试须在安装了 Flask/Flask-SocketIO 的 Web Python 环境中运行 `python3 -B publish/modules/src/xarm_ui/test_record_api.py`。
