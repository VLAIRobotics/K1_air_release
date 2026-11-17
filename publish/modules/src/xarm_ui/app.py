#!/usr/bin/env python3
"""
XArm 控制 Web UI 后端服务器
提供RESTful API和WebSocket接口用于机械臂控制和状态监控
"""

from flask import Flask, render_template, request, jsonify, Response
from flask_cors import CORS
from flask_socketio import SocketIO, emit
from robot_preview import register_robot_preview
import subprocess
import shlex
import re
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Dict, Optional, List
import logging

CAMERA_PREVIEW_TOPICS = {
    "cam_chest": "/cam_chest/cam_chest/color/image_raw",
    "cam_wrist_left": "/cam_wrist_left/cam_wrist_left/color/image_raw",
    "cam_wrist_right": "/cam_wrist_right/cam_wrist_right/color/image_raw",
}

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'xarm_control_secret_2026'

# 修改 Jinja2 分隔符，避免与 Vue.js 冲突
app.jinja_env.variable_start_string = '[['
app.jinja_env.variable_end_string = ']]'

CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# 全局进程管理
active_processes: Dict[str, subprocess.Popen] = {}
process_lock = threading.Lock()
# Serialize teleop transactions without nesting the ProcessManager lock.
teleop_lock = threading.Lock()
teleop_configs = {}
teleop_owners = {}
TELEOP_SIDES = ("left_arm", "right_arm")
TELEOP_STARTUP_GRACE = 0.2
TELEOP_TERM_TIMEOUT = 15.0
TELEOP_KILL_TIMEOUT = 5.0


def _read_process_identity(pid):
    """Read Linux identity without waitpid/poll, preserving an exited group leader."""
    try:
        text = (Path('/proc') / str(pid) / 'stat').read_text()
    except (FileNotFoundError, ProcessLookupError):
        return None
    fields = text[text.rfind(')') + 2:].split()
    return {'pid': int(pid), 'state': fields[0], 'pgid': int(fields[2]),
            'sid': int(fields[3]), 'start_ticks': int(fields[19])}


def _capture_teleop_owner(process):
    # Popen completed setsid before returning. Do not reap this leader while any
    # child is alive: its reserved PID is our non-reusable process-group anchor.
    owner = {'process': process, 'pgid': process.pid, 'sid': process.pid,
             'start_ticks': None, 'retired': False}
    try:
        identity = _read_process_identity(process.pid)
        if identity is None or identity['pgid'] != process.pid or identity['sid'] != process.pid:
            raise ValueError('无法核对新建遥操作进程组身份')
        owner['start_ticks'] = identity['start_ticks']
    except (OSError, ValueError, IndexError) as exc:
        owner['tracking_error'] = str(exc)
    return owner


def _teleop_group_state(owner):
    """Return running/stopped/unknown, never infer group exit from parent exit."""
    if owner.get('retired'):
        return {'state': 'stopped', 'members': []}
    try:
        anchor = _read_process_identity(owner['pgid'])
        if (owner['start_ticks'] is None or anchor is None or
                anchor['start_ticks'] != owner['start_ticks'] or
                anchor['pgid'] != owner['pgid'] or anchor['sid'] != owner['sid']):
            raise ValueError('无法确认原遥操作进程组身份，未向其他进程发送信号')
        members = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            identity = _read_process_identity(int(entry.name))
            if (identity and identity['pgid'] == owner['pgid'] and
                    identity['sid'] == owner['sid'] and identity['state'] not in ('Z', 'X')):
                members.append(identity['pid'])
        return {'state': 'running' if members else 'stopped', 'members': members}
    except (OSError, ValueError, IndexError) as exc:
        return {'state': 'unknown', 'members': [], 'message': str(exc)}


def _retire_teleop_owner(name, owner):
    """Caller holds process_lock and has confirmed no live group members."""
    if teleop_owners.get(name) is owner:
        owner['retired'] = True
        owner['process'].poll()  # Safe to reap only after the entire group exited.
        teleop_owners.pop(name, None)
        if active_processes.get(name) is owner['process']:
            active_processes.pop(name, None)
        teleop_configs.pop(name.removeprefix('teleop_'), None)


def _teleop_has_activity(entry):
    return bool(entry.get('running') or entry.get('pending'))


class CameraPreviewManager:
    """Subscribe ROS image topics and expose latest JPEG frames for the web UI."""

    def __init__(self):
        self._lock = threading.Lock()
        self._runtime_lock = threading.RLock()
        self._frames = {name: None for name in CAMERA_PREVIEW_TOPICS}
        self._subscriptions = {}
        self._rclpy = None
        self._node = None
        self._executor_thread = None

    def _ensure_runtime(self) -> Dict:
        with self._runtime_lock:
            return self._ensure_runtime_unlocked()

    def _ensure_runtime_unlocked(self) -> Dict:
        if self._node is not None:
            return {"success": True}
        try:
            import rclpy
            import cv2
            import numpy
            from rclpy.node import Node

            if not rclpy.ok():
                rclpy.init(args=None)
            self._rclpy = rclpy
            self._node = Node("xarm_ui_preview_node")
            self._executor_thread = threading.Thread(target=self._spin, daemon=True)
            self._executor_thread.start()
            return {"success": True}
        except Exception as exc:
            logger.error(f"初始化相机预览失败: {exc}")
            return {"success": False, "message": str(exc)}

    def _spin(self):
        while self._node is not None and self._rclpy is not None:
            try:
                self._rclpy.spin_once(self._node, timeout_sec=0.1)
            except Exception as exc:
                logger.error(f"相机预览 spin 失败: {exc}")
                time.sleep(0.2)

    @staticmethod
    def _image_to_bgr(msg):
        """Decode 8-bit color images, including row padding, without cv_bridge."""
        import cv2
        import numpy as np

        formats = {
            "bgr8": (3, None),
            "rgb8": (3, cv2.COLOR_RGB2BGR),
            "bgra8": (4, cv2.COLOR_BGRA2BGR),
            "rgba8": (4, cv2.COLOR_RGBA2BGR),
            "mono8": (1, cv2.COLOR_GRAY2BGR),
        }
        if msg.encoding not in formats:
            raise ValueError(f"不支持的彩色预览编码: {msg.encoding}")
        channels, conversion = formats[msg.encoding]
        row_bytes = msg.width * channels
        if msg.width <= 0 or msg.height <= 0 or msg.step < row_bytes:
            raise ValueError("图像尺寸或步长无效")
        pixels = np.frombuffer(msg.data, dtype=np.uint8)
        if pixels.size < msg.height * msg.step:
            raise ValueError("图像数据长度不足")
        rows = pixels[:msg.height * msg.step].reshape(msg.height, msg.step)
        shape = (msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width)
        frame = np.ascontiguousarray(rows[:, :row_bytes].reshape(shape))
        return cv2.cvtColor(frame, conversion) if conversion is not None else frame

    def _make_callback(self, camera_name: str):
        def _callback(msg):
            try:
                import cv2
                frame = self._image_to_bgr(msg)
                ok, encoded = cv2.imencode(".jpg", frame)
                if ok:
                    with self._lock:
                        if camera_name in self._subscriptions:
                            self._frames[camera_name] = encoded.tobytes()
            except Exception as exc:
                logger.error(f"{camera_name} 图像转换失败: {exc}")
        return _callback

    def start(self, camera_names: Optional[List[str]] = None) -> Dict:
        runtime = self._ensure_runtime()
        if not runtime.get("success"):
            return runtime
        try:
            from sensor_msgs.msg import Image
            from rclpy.qos import qos_profile_sensor_data
            targets = camera_names or list(CAMERA_PREVIEW_TOPICS.keys())
            with self._lock:
                for camera_name in targets:
                    if camera_name not in CAMERA_PREVIEW_TOPICS:
                        continue
                    if camera_name in self._subscriptions:
                        continue
                    sub = self._node.create_subscription(
                        Image,
                        CAMERA_PREVIEW_TOPICS[camera_name],
                        self._make_callback(camera_name),
                        qos_profile_sensor_data,
                    )
                    self._subscriptions[camera_name] = sub
            return {"success": True, "message": "相机预览已启动"}
        except Exception as exc:
            logger.error(f"启动相机预览失败: {exc}")
            return {"success": False, "message": str(exc)}

    def stop_all(self):
        with self._lock:
            for camera_name, sub in list(self._subscriptions.items()):
                try:
                    self._node.destroy_subscription(sub)
                except Exception as exc:
                    logger.warning(f"停止 {camera_name} 预览订阅失败: {exc}")
                self._frames[camera_name] = None
            self._subscriptions.clear()

    def status(self, camera_name: str) -> Dict:
        with self._lock:
            return {
                "success": True,
                "subscribed": camera_name in self._subscriptions,
                "available": self._frames.get(camera_name) is not None,
                "topic": CAMERA_PREVIEW_TOPICS.get(camera_name, ""),
            }

    def frame(self, camera_name: str):
        with self._lock:
            return self._frames.get(camera_name)


preview_manager = CameraPreviewManager()
robot_preview_manager = register_robot_preview(app, ros_manager=preview_manager)

# 项目根目录
PROJECT_ROOT = Path(__file__).parent.parent.parent
TELEOP_DIR = PROJECT_ROOT / "src" / "xarm_teleop"
TELEOP_SCRIPT_DIR = PROJECT_ROOT / "src" / "xarm_teleop" / "script"
COLLECTOR_DIR = PROJECT_ROOT.parent / "lerobot_collector"
XARM_CAN_LIBEXEC_DIR = PROJECT_ROOT.parent / "xarm_can" / "package" / "libexec"

RECORD_ARM_SIDES = ("left", "right")


def recorder_process_name(arm_side: str) -> str:
    return f"recorder_{arm_side}_arm"


def recorder_python() -> Path:
    configured = os.environ.get("XARM_UI_RECORDER_PYTHON")
    if configured:
        return Path(configured).expanduser()
    conda_python = Path("/opt/miniconda/envs/lerobot/bin/python3")
    return conda_python if conda_python.is_file() else Path(sys.executable)


class ProcessManager:
    """管理后台进程"""
    
    @staticmethod
    def start_process(
        name: str, 
        command: List[str], 
        cwd: Optional[str] = None,
        env: Optional[Dict] = None,
        enable_stdin: bool = False
    ) -> Dict:
        """启动一个后台进程
        
        Args:
            name: 进程名称
            command: 命令列表
            cwd: 工作目录
            env: 环境变量
            enable_stdin: 是否启用stdin通信（用于交互式控制）
        """
        with process_lock:
            if name in teleop_owners:
                owner = teleop_owners[name]
                if _teleop_group_state(owner)['state'] != 'stopped':
                    return {"success": False, "message": f"进程组 {name} 仍在运行或尚未确认退出"}
                _retire_teleop_owner(name, owner)
            elif name in active_processes:
                if name in ('teleop_left_arm', 'teleop_right_arm') or active_processes[name].poll() is None:
                    return {"success": False, "message": f"进程 {name} 已在运行或身份待确认"}
            
            try:
                # 合并环境变量
                process_env = os.environ.copy()
                if env:
                    process_env.update(env)
                
                # 启动进程
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    stdin=subprocess.PIPE if enable_stdin else subprocess.DEVNULL,
                    cwd=cwd,
                    env=process_env,
                    text=True,
                    bufsize=1,
                    preexec_fn=os.setsid  # 创建新进程组
                )
                
                active_processes[name] = process
                if name in ('teleop_left_arm', 'teleop_right_arm'):
                    teleop_owners[name] = _capture_teleop_owner(process)
                logger.info(f"✅ 启动进程: {name}, PID: {process.pid}")
                
                # 启动日志监听线程
                threading.Thread(
                    target=ProcessManager._stream_output,
                    args=(name, process),
                    daemon=True
                ).start()
                
                return {
                    "success": True, 
                    "message": f"进程 {name} 启动成功",
                    "pid": process.pid
                }
                
            except Exception as e:
                logger.error(f"❌ 启动进程失败: {name}, 错误: {e}")
                return {"success": False, "message": str(e)}
    
    @staticmethod
    def stop_process(name: str) -> Dict:
        """停止一个后台进程"""
        if name in ('teleop_left_arm', 'teleop_right_arm'):
            with teleop_lock:
                return _stop_owned_teleop(name.removeprefix('teleop_'))
        fallback_patterns = {
            'camera_node': r'multi_realsense|realsense2_camera|ros2 launch multi_realsense',
            'teleop_right_arm': r'unilateral_control.*right_arm|launch_unilateral.*right_arm',
            'teleop_left_arm': r'unilateral_control.*left_arm|launch_unilateral.*left_arm',
            'recorder_right_arm': r'xarm_ros2_record.py.*--arm-side right',
            'recorder_left_arm': r'xarm_ros2_record.py.*--arm-side left',
        }

        with process_lock:
            if name not in active_processes or active_processes[name].poll() is not None:
                # 从 active_processes 清理残留
                if name in active_processes:
                    del active_processes[name]
                # 尝试 pgrep 兜底
                pattern = fallback_patterns.get(name)
                if pattern:
                    try:
                        result = subprocess.run(
                            ['pgrep', '-f', pattern],
                            capture_output=True, text=True, timeout=2
                        )
                        pids = [int(p) for p in result.stdout.strip().split('\n') if p.strip().isdigit()]
                        if pids:
                            for pid in pids:
                                try:
                                    os.killpg(os.getpgid(pid), signal.SIGTERM)
                                except ProcessLookupError:
                                    os.kill(pid, signal.SIGTERM)
                            logger.info(f"✅ 通过 pgrep 停止进程: {name} (pids: {pids})")
                            return {"success": True, "message": f"进程 {name} 已停止"}
                    except Exception as e:
                        logger.error(f"pgrep fallback 失败: {e}")
                return {"success": False, "message": f"进程 {name} 不存在"}

            process = active_processes[name]
            try:
                # 向进程组发送SIGTERM
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)

                # 等待进程结束（recorder cleanup 需要时间写入数据）
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
                    process.wait()

                del active_processes[name]
                logger.info(f"✅ 停止进程: {name}")
                return {"success": True, "message": f"进程 {name} 已停止"}

            except Exception as e:
                logger.error(f"❌ 停止进程失败: {name}, 错误: {e}")
                return {"success": False, "message": str(e)}
    
    @staticmethod
    def get_status() -> Dict:
        """获取所有进程状态"""
        with process_lock:
            status = {}
            for name, process in list(active_processes.items()):
                owner = teleop_owners.get(name)
                if owner is not None:
                    group = _teleop_group_state(owner)
                    status[name] = {'running': group['state'] == 'running',
                                    'pending': group['state'] == 'unknown',
                                    'pid': owner['pgid'], 'member_pids': group['members']}
                    if group['state'] == 'unknown':
                        status[name]['message'] = group['message']
                    if group['state'] == 'stopped':
                        _retire_teleop_owner(name, owner)
                    continue
                if name in ('teleop_left_arm', 'teleop_right_arm'):
                    status[name] = {'running': False, 'pending': True, 'pid': process.pid,
                                    'message': '进程组身份未记录，无法确认该臂状态'}
                    continue
                poll_result = process.poll()
                if poll_result is None:
                    status[name] = {"running": True, "pid": process.pid}
                else:
                    status[name] = {"running": False, "exit_code": poll_result}
                    # 清理已结束的进程
                    del active_processes[name]
            
            # 兼容：页面刷新后进程仍在，但active_processes已丢失
            def _find_pids(pattern: str) -> List[int]:
                try:
                    result = subprocess.run(
                        ['pgrep', '-f', pattern],
                        capture_output=True,
                        text=True,
                        timeout=2
                    )
                    if result.returncode != 0:
                        return []
                    return [int(pid) for pid in result.stdout.strip().split('\n') if pid.strip().isdigit()]
                except Exception:
                    return []
            
            fallback_checks = {
                'camera_node': r'multi_realsense|realsense2_camera|ros2 launch multi_realsense',
                'teleop_right_arm': r'unilateral_control.*right_arm|launch_unilateral.*right_arm',
                'teleop_left_arm': r'unilateral_control.*left_arm|launch_unilateral.*left_arm',
                'recorder_right_arm': r'xarm_ros2_record.py.*--arm-side right',
                'recorder_left_arm': r'xarm_ros2_record.py.*--arm-side left'
            }
            
            for name, pattern in fallback_checks.items():
                if name in status:
                    continue
                pids = _find_pids(pattern)
                if pids:
                    status[name] = {"running": True, "pid": pids[0]}
            
            for side in TELEOP_SIDES:
                entry = status.setdefault('teleop_' + side, {'running': False})
                process = active_processes.get('teleop_' + side)
                entry['managed'] = ('teleop_' + side) in teleop_owners
                if entry['managed'] and side in teleop_configs:
                    entry['config'] = dict(teleop_configs[side])
            running_count = sum(bool(status['teleop_' + side]['running']) for side in TELEOP_SIDES)
            status['teleop_dual_arm'] = {'running': running_count == 2, 'partial': running_count == 1,
                                        'running_arms': [side for side in TELEOP_SIDES if status['teleop_' + side]['running']]}
            return status
    
    @staticmethod
    def _stream_output(name: str, process: subprocess.Popen):
        """流式读取进程输出并通过WebSocket发送"""
        def _classify(msg: str, default_type: str) -> str:
            """根据 ROS2 日志前缀判断实际类型"""
            if '[ERROR]' in msg or '[FATAL]' in msg:
                return 'stderr'
            if '[WARN]' in msg:
                return 'stdwarn'
            if '[INFO]' in msg:
                return 'stdout'
            return default_type

        def _read_stream(stream, stream_type: str):
            try:
                for line in iter(stream.readline, ''):
                    if line:
                        msg = line.strip()
                        socketio.emit('process_log', {
                            'process': name,
                            'type': _classify(msg, stream_type),
                            'message': msg
                        })
            except Exception as e:
                logger.error(f"日志流错误 {name} ({stream_type}): {e}")
        
        try:
            if process.stdout:
                threading.Thread(
                    target=_read_stream,
                    args=(process.stdout, 'stdout'),
                    daemon=True
                ).start()
            if process.stderr:
                threading.Thread(
                    target=_read_stream,
                    args=(process.stderr, 'stderr'),
                    daemon=True
                ).start()
        except Exception as e:
            logger.error(f"日志流错误 {name}: {e}")


# ==================== REST API 路由 ====================

@app.route('/')
def index():
    """主页"""
    return render_template('index-v1.html')


@app.route('/api/system/info', methods=['GET'])
def system_info():
    """获取系统信息"""
    return jsonify({
        "project_root": str(PROJECT_ROOT),
        "teleop_scripts": str(TELEOP_SCRIPT_DIR),
        "collector_dir": str(COLLECTOR_DIR),
        "ros_distro": os.environ.get('ROS_DISTRO', 'unknown')
    })


@app.route('/api/process/status', methods=['GET'])
def process_status():
    """获取所有进程状态"""
    return jsonify(ProcessManager.get_status())


def _teleop_sides(arm_side):
    if arm_side == 'dual_arm':
        return TELEOP_SIDES
    if arm_side in TELEOP_SIDES:
        return (arm_side,)
    raise ValueError('arm_side 必须为 left_arm、right_arm 或 dual_arm')


def _teleop_request(data):
    if not isinstance(data, dict):
        raise ValueError('请求必须是 JSON 对象')
    sides = _teleop_sides(data.get('arm_side', 'right_arm'))
    use_ros2 = data.get('use_ros2', False)
    if not isinstance(use_ros2, bool):
        raise ValueError('use_ros2 必须为布尔值')
    configs = {}
    for side in sides:
        # Preserve the existing single-arm API and its defaults.
        prefix = side.split('_')[0]
        leader_key = prefix + '_leader_can' if len(sides) == 2 else 'leader_can'
        follower_key = prefix + '_follower_can' if len(sides) == 2 else 'follower_can'
        leader_default = 'can0' if side == 'right_arm' else 'can2'
        follower_default = 'can1' if side == 'right_arm' else 'can3'
        leader = data.get(leader_key, leader_default if len(sides) == 1 else None)
        follower = data.get(follower_key, follower_default if len(sides) == 1 else None)
        for interface in (leader, follower):
            if not isinstance(interface, str) or not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.-]{0,14}', interface):
                raise ValueError('请填写有效 CAN 接口名（最多15字符）')
        configs[side] = {'leader_can': leader, 'follower_can': follower, 'use_ros2': use_ros2}
    interfaces = [config[key] for config in configs.values() for key in ('leader_can', 'follower_can')]
    if len(set(interfaces)) != len(interfaces):
        raise ValueError('每个主臂、从臂必须使用不同 CAN 接口；双臂需要4个不同接口')
    return sides, configs


def _wait_teleop_group(owner, timeout):
    deadline = time.monotonic() + timeout
    while True:
        state = _teleop_group_state(owner)
        if state['state'] != 'running' or time.monotonic() >= deadline:
            return state
        time.sleep(min(0.05, max(0, deadline - time.monotonic())))


def _signal_owned_teleop(owner, sig):
    # Keep the leader unreaped from identity validation through signalling.
    # get_status/retire use the same lock; group waits stay outside this lock.
    with process_lock:
        state = _teleop_group_state(owner)
        if state['state'] == 'unknown':
            raise ValueError(state['message'])
        if state['state'] == 'running':
            os.killpg(owner['pgid'], sig)


def _stop_owned_teleop(side):
    name = 'teleop_' + side
    with process_lock:
        owner = teleop_owners.get(name)
    if owner is None:
        return {'success': False, 'message': '此 UI 未跟踪该臂进程组，不会终止其他会话'}
    # Never wait/reap only the bash parent: its child may still control motors.
    try:
        state = _teleop_group_state(owner)
        if state['state'] == 'unknown':
            return {'success': False, 'pending': True, 'message': state['message']}
        if state['state'] == 'running':
            try:
                _signal_owned_teleop(owner, signal.SIGTERM)
            except ProcessLookupError:
                pass
            state = _wait_teleop_group(owner, TELEOP_TERM_TIMEOUT)
            if state['state'] == 'running':
                try:
                    _signal_owned_teleop(owner, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                state = _wait_teleop_group(owner, TELEOP_KILL_TIMEOUT)
        if state['state'] != 'stopped':
            return {'success': False, 'pending': True,
                    'message': state.get('message', '遥操作进程组尚未退出，已保留跟踪记录')}
        with process_lock:
            _retire_teleop_owner(name, owner)
        return {'success': True, 'message': '该臂遥操作进程组已全部退出'}
    except Exception as exc:
        return {'success': False, 'pending': True, 'message': str(exc)}


@app.route('/api/teleop/start', methods=['POST'])
def start_teleop():
    """Start one arm or both, keeping each existing arm session independent."""
    try:
        sides, configs = _teleop_request(request.get_json(silent=True))
    except ValueError as exc:
        return jsonify({'success': False, 'message': str(exc)}), 400
    with teleop_lock:
        status = ProcessManager.get_status()
        with process_lock:
            owned = {side: active_processes.get('teleop_' + side) for side in TELEOP_SIDES}
            current_configs = dict(teleop_configs)
        for side in TELEOP_SIDES:
            if not _teleop_has_activity(status.get('teleop_' + side, {})):
                continue
            if status['teleop_' + side].get('pending'):
                return jsonify({'success': False, 'message': '原遥操作进程组状态待确认，请先处理退出状态'}), 409
            current = current_configs.get(side)
            if owned[side] is None or current is None:
                return jsonify({'success': False, 'message': '检测到未由此 UI 管理的遥操作，请先在原终端退出'}), 409
            if side in configs:
                if current != configs[side]:
                    return jsonify({'success': False, 'message': '该臂已按其他配置运行，请先停止该臂再修改'}), 409
            else:
                busy = {current['leader_can'], current['follower_can']}
                if any(busy.intersection((c['leader_can'], c['follower_can'])) for c in configs.values()):
                    return jsonify({'success': False, 'message': 'CAN 接口与正在运行的另一臂冲突'}), 409
        # Validate all scripts before starting either side.
        script_paths = {}
        for side in sides:
            filename = 'launch_unilateral_ros2.sh' if configs[side]['use_ros2'] else 'launch_unilateral.sh'
            script_paths[side] = TELEOP_SCRIPT_DIR / filename
            if not script_paths[side].exists():
                return jsonify({'success': False, 'message': '遥操作启动脚本不存在: ' + filename}), 400
        started, reused, results = [], [], {}
        for side in sides:
            if status.get('teleop_' + side, {}).get('running'):
                reused.append(side)
                results[side] = {'success': True, 'already_running': True}
                continue
            config = configs[side]
            result = ProcessManager.start_process(
                name='teleop_' + side,
                command=['bash', str(script_paths[side]), side, config['leader_can'], config['follower_can']],
                cwd=str(TELEOP_DIR))
            if result.get('success'):
                started.append(side)
                with process_lock:
                    teleop_configs[side] = dict(config)
                # Catch immediate script failures; process creation is not a robot-ready acknowledgement.
                time.sleep(TELEOP_STARTUP_GRACE)
                with process_lock:
                    owner = teleop_owners.get('teleop_' + side)
                    group = _teleop_group_state(owner) if owner else {'state': 'unknown'}
                    if group['state'] != 'running':
                        result = {'success': False, 'message': '启动进程组未运行或身份待确认，请查看该臂日志'}
            results[side] = result
            if not result.get('success'):
                rollback = {arm: _stop_owned_teleop(arm) for arm in reversed(started)}
                return jsonify({'success': False, 'message': '遥操作启动失败，仅回退本次新启动的进程',
                                'arms': results, 'reused': reused, 'rollback': rollback}), 500
        return jsonify({'success': True, 'message': '遥操作进程已启动；请查看各臂日志确认设备状态',
                        'arms': results, 'started': started, 'reused': reused})


@app.route('/api/teleop/stop', methods=['POST'])
def stop_teleop():
    data = request.get_json(silent=True)
    try:
        if not isinstance(data, dict):
            raise ValueError('请求必须是 JSON 对象')
        sides = _teleop_sides(data.get('arm_side', 'right_arm'))
    except ValueError as exc:
        return jsonify({'success': False, 'message': str(exc)}), 400
    with teleop_lock:
        status = ProcessManager.get_status()
        results = {side: (_stop_owned_teleop(side) if _teleop_has_activity(status.get('teleop_' + side, {}))
                          else {'success': True, 'already_stopped': True, 'message': '该臂未运行'}) for side in sides}
    success = all(result.get('success') for result in results.values())
    return jsonify({'success': success, 'arms': results,
                    'message': '所选遥操作进程已退出' if success else '部分进程未退出或不属于此 UI，请查看各臂状态'})


@app.route('/api/can/status', methods=['GET'])
def can_status():
    """检查 CAN 接口状态"""
    try:
        result = subprocess.run(
            ['ip', 'link', 'show'],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        # 过滤出 CAN 接口信息
        lines = result.stdout.split('\n')
        can_interfaces = []
        current_interface = None
        
        for line in lines:
            line = line.strip()
            if not line:
                continue
                
            # 检测接口行（包含 ":"）
            if ': can' in line:
                parts = line.split(':')
                if len(parts) >= 2:
                    # 提取接口名
                    iface_name = parts[1].strip().split()[0]
                    
                    # 检查是否为 UP 状态
                    is_up = 'UP' in line and 'state UP' in line
                    
                    # 构造简洁的状态信息
                    if 'state UP' in line:
                        status = 'UP - 运行中'
                    elif 'state DOWN' in line:
                        status = 'DOWN - 未启动'
                    else:
                        status = '未知状态'
                    
                    # 提取 MTU 信息
                    if 'mtu' in line:
                        mtu_match = line.split('mtu')[1].split()[0]
                        status += f' (MTU: {mtu_match})'
                    
                    can_interfaces.append({
                        'name': iface_name,
                        'status': status,
                        'is_up': is_up
                    })
        
        return jsonify({
            "success": True,
            "interfaces": can_interfaces,
            "output": result.stdout
        })
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/api/can/setup', methods=['POST'])
def can_setup():
    """配置 CAN 接口"""
    try:
        script_path = XARM_CAN_LIBEXEC_DIR / 'setup_can_interfaces.sh'
        
        if not script_path.exists():
            return jsonify({
                "success": False,
                "message": f"配置脚本不存在: {script_path}"
            })
        
        # 已经是 root 直接跑脚本；非 root 才需要 pkexec 弹图形密码提权
        # （pkexec 依赖 polkit D-Bus 服务，纯命令行/容器环境下通常不可用）
        cmd = ['bash', str(script_path)] if os.geteuid() == 0 else ['pkexec', 'bash', str(script_path)]
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=60,
            env=os.environ.copy()
        )

        if result.returncode != 0 and 'not authorized' in result.stderr.lower():
            # 如果 pkexec 失败，返回手动配置提示
            return jsonify({
                "success": False,
                "message": "需要管理员权限。请在终端手动运行：sudo " + str(script_path),
                "error": result.stderr,
                "manual_command": f"sudo {script_path}"
            })
        
        return jsonify({
            "success": result.returncode == 0,
            "output": result.stdout,
            "error": result.stderr,
            "message": "CAN 接口配置成功" if result.returncode == 0 else "CAN 接口配置失败"
        })
    except subprocess.TimeoutExpired:
        return jsonify({
            "success": False,
            "message": "配置超时，可能需要手动授权"
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "message": f"配置失败: {str(e)}。请在终端手动运行：sudo {script_path}",
            "manual_command": f"sudo {script_path}"
        })


@app.route('/api/can/setzero', methods=['POST'])
def can_setzero():
    """设置机械臂零点"""
    try:
        data = request.get_json() or {}
        can_if = data.get('can_if', 'can0')
        script_path = XARM_CAN_LIBEXEC_DIR / 'set_zero.sh'

        if not script_path.exists():
            return jsonify({"success": False, "message": f"脚本不存在: {script_path}"})

        result = subprocess.run(
            ['bash', str(script_path), can_if, '--all'],
            capture_output=True, text=True, timeout=30,
            env=os.environ.copy()
        )

        return jsonify({
            "success": result.returncode == 0,
            "output": result.stdout,
            "error": result.stderr,
            "message": f"{can_if} 零点设置完成" if result.returncode == 0 else f"{can_if} 零点设置失败"
        })
    except subprocess.TimeoutExpired:
        return jsonify({"success": False, "message": "零点设置超时"})
    except Exception as e:
        return jsonify({"success": False, "message": f"零点设置失败: {str(e)}"})


@app.route('/api/camera/check', methods=['GET'])
def check_cameras():
    """检查相机"""
    try:
        workspace_setup = str(PROJECT_ROOT / "install" / "setup.bash")
        result = subprocess.run(
            ['bash', '-c', f'source /opt/ros/humble/setup.bash && source {shlex.quote(workspace_setup)} && ros2 run multi_realsense check_cameras.py'],
            capture_output=True,
            text=True,
            timeout=20
        )
        
        output = result.stdout
        error = result.stderr
        
        # 记录原始输出以便调试
        logger.info(f"检查相机 - 返回码: {result.returncode}")
        logger.info(f"检查相机 - stdout: {output[:200] if output else '(空)'}")
        logger.info(f"检查相机 - stderr: {error[:200] if error else '(空)'}")
        
        # Only a successful enumeration is a positive result; "未检测到" is not success.
        import re
        count_match = re.search(r'✅ 检测到\s+(\d+)\s+个相机', output)
        has_camera = result.returncode == 0 and count_match and int(count_match.group(1)) > 0
        
        if has_camera:
            # 提取关键信息：相机数量和每个相机的信息
            import re
            
            # 提取相机数量
            camera_count_match = re.search(r'检测到\s+(\d+)\s+个相机', output)
            camera_count = camera_count_match.group(1) if camera_count_match else '未知'
            
            # 提取每个相机的信息
            camera_info = []
            camera_list = []  # 用于前端下拉选择
            camera_blocks = re.findall(r'相机\s+\d+:\s+名称:\s+(.+?)\s+序列号:\s+(\w+)', output, re.DOTALL)
            for idx, (name, serial) in enumerate(camera_blocks, 1):
                name = name.strip()
                camera_info.append(f"相机 {idx}: {name} (序列号: {serial})")
                camera_list.append({
                    "name": name,
                    "serial": serial
                })
            
            # 构造简洁的输出
            summary = f"✅ 检测到 {camera_count} 个相机\n\n" + "\n".join(camera_info)
            
            return jsonify({
                "success": True,
                "output": summary,
                "cameras": camera_list,  # 返回相机列表供前端选择
                "raw_output": output,  # 保留原始输出以备需要
                "error": error
            })
        else:
            # Do not turn missing tools, packages, or workspace errors into "no camera".
            no_camera = "❌ 未检测到 RealSense 相机" in output
            details = "\n".join(part.strip() for part in (output, error) if part.strip())
            message = ("未检测到 RealSense 相机，请检查连接、权限和驱动"
                       if no_camera else "相机检测命令执行失败，请查看以下详细信息")
            return jsonify({
                "success": False,
                "output": message + ("\n\n" + details if details else ""),
                "message": message,
                "raw_output": output,
                "error": error,
                "returncode": result.returncode,
                "no_camera": no_camera,
            })
    except subprocess.TimeoutExpired:
        return jsonify({
            "success": False,
            "message": "检查相机超时（20秒）",
            "error": "命令执行超时，请检查系统状态"
        })
    except Exception as e:
        logger.error(f"检查相机异常: {e}")
        return jsonify({
            "success": False,
            "message": str(e),
            "error": "执行检查命令时出错"
        })


@app.route('/api/camera/launch', methods=['POST'])
def launch_cameras():
    """启动相机节点"""
    data = request.json
    camera_config = data.get('config', 'multi')  # 'single' or 'multi'
    
    # 创建临时启动脚本
    import tempfile
    workspace_setup = str(PROJECT_ROOT / "install" / "setup.bash")
    
    def serial_value(value):
        serial = str(value or '').strip()
        if serial.startswith('_'):
            serial = serial[1:]
        if serial and not serial.isdecimal():
            raise ValueError("相机序列号应为数字，可带一个前导下划线")
        return serial

    try:
        if camera_config == 'multi':
            roles = {'serialChest': 'serial_chest', 'serialLeft': 'serial_left',
                     'serialRight': 'serial_right'}
            selected = [(arg, serial_value(data.get(key))) for key, arg in roles.items()]
            selected = [(arg, serial) for arg, serial in selected if serial]
            if not selected:
                return jsonify({"success": False, "message": "请至少选择一个相机"})
            if len({serial for _, serial in selected}) != len(selected):
                return jsonify({"success": False, "message": "同一相机不能分配给多个位置"})
            command_args = ['ros2', 'launch', 'multi_realsense', 'multi_cameras.launch.py']
            command_args += [f'{arg}:=_{serial}' for arg, serial in selected]
        elif camera_config == 'single':
            serial = serial_value(data.get('serialNo'))
            if not serial:
                return jsonify({"success": False, "message": "请先检查相机并选择序列号"})
            command_args = ['ros2', 'launch', 'multi_realsense', 'single_camera.launch.py',
                            f"camera_name:={data.get('cameraName', 'test_cam')}",
                            f'serial_no:=_{serial}']
        else:
            return jsonify({"success": False, "message": "未知相机配置"}), 400
    except ValueError as exc:
        return jsonify({"success": False, "message": str(exc)}), 400

    # Quote paths and arguments once; shell quotes must not enter the serial value.
    launch_script = ("#!/bin/bash\nset -e\n"
                     "source /opt/ros/humble/setup.bash\n"
                     f"source {shlex.quote(workspace_setup)}\n"
                     "ros2 pkg prefix multi_realsense >/dev/null\n"
                     f"exec {shlex.join(command_args)}\n")

    # 写入临时脚本
    with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
        f.write(launch_script)
        script_path = f.name
    
    # 添加执行权限
    os.chmod(script_path, 0o755)
    
    logger.info(f"启动相机，使用临时脚本: {script_path}")
    logger.info(f"工作空间路径: {workspace_setup}")
    command = ['bash', script_path]
    
    result = ProcessManager.start_process(
        name='camera_node',
        command=command
    )
    
    return jsonify(result)


@app.route('/api/camera/stop', methods=['POST'])
def stop_cameras():
    """停止相机节点"""
    preview_manager.stop_all()
    result = ProcessManager.stop_process('camera_node')
    
    # 额外清理：防止 ros2 launch 派生进程残留
    try:
        subprocess.run(
            ['pkill', '-9', '-f', 'multi_realsense|realsense2_camera|single_camera.launch.py|multi_cameras.launch.py'],
            capture_output=True,
            timeout=5
        )
        logger.info("已清理所有 multi_realsense / realsense2_camera 进程")
    except Exception as e:
        logger.warning(f"清理相机进程时出错: {e}")
    
    return jsonify(result)


@app.route('/api/camera/preview/start', methods=['POST'])
def start_camera_preview():
    """启动网页相机预览订阅。"""
    data = request.json or {}
    camera_names = data.get("camera_names")
    if not isinstance(camera_names, list):
        camera_names = None
    return jsonify(preview_manager.start(camera_names))


@app.route('/api/camera/preview/status', methods=['GET'])
def camera_preview_status():
    """返回指定相机的网页预览状态。"""
    camera_name = request.args.get("camera_name", "cam_chest")
    return jsonify(preview_manager.status(camera_name))


@app.route('/api/camera/preview/stream', methods=['GET'])
def camera_preview_stream():
    """输出 MJPEG 预览流。"""
    camera_name = request.args.get("camera_name", "cam_chest")
    if camera_name not in CAMERA_PREVIEW_TOPICS:
        return jsonify({"success": False, "message": "未知相机"}), 404

    def generate():
        while True:
            payload = preview_manager.frame(camera_name)
            if payload is None:
                time.sleep(0.05)
                continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + payload + b"\r\n"
            )
            time.sleep(0.03)

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route('/api/product/capabilities', methods=['GET'])
def product_capabilities():
    return jsonify({
        "recording": (COLLECTOR_DIR / "xarm_ros2_record.py").is_file(),
        "managed_cameras": (PROJECT_ROOT / "src" / "multi_realsense").is_dir(),
    })


@app.before_request
def require_installed_product_module():
    if request.path.startswith('/api/record/') and not (COLLECTOR_DIR / "xarm_ros2_record.py").is_file():
        return jsonify({"success": False, "message": "当前版本未包含机械臂数据采集模块，请使用 Pro 或 Max。"}), 409
    if request.path in ('/api/camera/check', '/api/camera/launch') and not (PROJECT_ROOT / "src" / "multi_realsense").is_dir():
        return jsonify({"success": False, "message": "当前版本未包含多相机管理模块，请使用 Pro 或 Max。"}), 409


@app.route('/api/record/start', methods=['POST'])
def start_recording():
    """启动当前 K1Max 单臂 8D Recorder；Recorder 只订阅 ROS。"""
    data = request.get_json(silent=True) or {}
    if not isinstance(data, dict):
        return jsonify({"success": False, "message": "录制配置必须是 JSON 对象"}), 400

    arm_side = data.get('arm_side')
    if arm_side not in RECORD_ARM_SIDES:
        return jsonify({"success": False, "message": "arm_side 必须为 left 或 right"}), 400
    repo_id = str(data.get('repo_id') or f'local/k1_{arm_side}_arm').strip()
    task = str(data.get('task') or f'K1 {arm_side} arm').strip()
    fps = data.get('fps', 30)
    use_wrist_camera = data.get('use_wrist_camera', False)
    if not isinstance(fps, int) or isinstance(fps, bool) or not 1 <= fps <= 120:
        return jsonify({"success": False, "message": "fps 必须为 1～120 的整数"}), 400
    if not isinstance(use_wrist_camera, bool):
        return jsonify({"success": False, "message": "use_wrist_camera 必须为布尔值"}), 400

    ros_setup = Path(os.environ.get('ROS_SETUP', '/opt/ros/humble/setup.bash'))
    workspace_setup = PROJECT_ROOT / 'install' / 'setup.bash'
    recorder_script = COLLECTOR_DIR / 'xarm_ros2_record.py'
    python = recorder_python()
    for path in (ros_setup, workspace_setup, recorder_script):
        if not path.is_file():
            return jsonify({"success": False, "message": f"录制依赖不存在: {path}"}), 400
    if not python.is_file() or not os.access(python, os.X_OK):
        return jsonify({"success": False, "message": f"LeRobot Python 不可执行: {python}"}), 400

    root_value = str(data.get('root') or '').strip()
    if root_value:
        root_path = Path(root_value).expanduser()
        if not root_path.is_absolute():
            return jsonify({"success": False, "message": "保存路径须为绝对路径，或留空自动生成"}), 400
    else:
        base = Path(os.environ.get('XARM_UI_DATASET_BASE', str(Path.home() / 'lerobot_datasets'))).expanduser()
        root_path = base / f'k1_{arm_side}_arm_{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns() % 1000000:06d}'
    root_path = root_path.resolve()
    if root_path.exists():
        return jsonify({"success": False, "message": f"数据目录已存在，不会覆盖: {root_path}"}), 400
    try:
        root_path.parent.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        return jsonify({"success": False, "message": f"无法创建数据目录的父目录: {exc}"}), 400
    if not os.access(root_path.parent, os.W_OK):
        return jsonify({"success": False, "message": f"数据目录不可写: {root_path.parent}"}), 400

    # 所有用户输入都作为独立 argv 传入；shell 只负责加载 ROS 环境。
    command = [
        'bash', '-c', 'source "$1" && source "$2" && shift 2 && exec "$@"',
        'xarm-ui-recorder', str(ros_setup), str(workspace_setup),
        str(python), '-B', '-u', str(recorder_script),
        '--arm-side', arm_side, '--repo-id', repo_id, '--root', str(root_path),
        '--fps', str(fps), '--task', task,
    ]
    if use_wrist_camera:
        command.append('--use-wrist-camera')

    result = ProcessManager.start_process(
        name=recorder_process_name(arm_side),
        command=command,
        cwd=str(COLLECTOR_DIR),
        enable_stdin=True,
    )
    if result.get('success'):
        result['root'] = str(root_path)
        result['message'] = f"录制进程已启动，等待 Recorder ready；数据目录: {root_path}"
    return jsonify(result)


@app.route('/api/record/stop', methods=['POST'])
def stop_recording():
    """发送 q 正常结束 Recorder；未保存的 buffer 会被丢弃。"""
    data = request.get_json(silent=True) or {}
    arm_side = data.get('arm_side') if isinstance(data, dict) else None
    if arm_side not in RECORD_ARM_SIDES:
        return jsonify({"success": False, "message": "arm_side 必须为 left 或 right"}), 400
    process_name = recorder_process_name(arm_side)
    with process_lock:
        process = active_processes.get(process_name)
        if process is None or process.poll() is not None or process.stdin is None:
            return jsonify({"success": False, "message": "Web 未持有该 Recorder；请检查进程，勿强制结束未保存数据"})
        try:
            process.stdin.write('q\n')
            process.stdin.flush()
        except OSError as exc:
            return jsonify({"success": False, "message": f"无法向 Recorder 发送退出命令: {exc}"})
    try:
        exit_code = process.wait(timeout=30)
    except subprocess.TimeoutExpired:
        return jsonify({"success": False, "message": "已发送 q，Recorder 仍在收尾；请等待日志，不要重复启动"})
    with process_lock:
        if active_processes.get(process_name) is process:
            del active_processes[process_name]
    return jsonify({
        "success": exit_code == 0,
        "message": f"Recorder 已退出（exit={exit_code}）；未保存的 buffer 已丢弃",
    })


@app.route('/api/record/control', methods=['POST'])
def control_recording():
    """控制数据录制的episode操作
    
    支持的命令:
        - 'r': 开始/暂停录制当前 episode
        - 's': 保存当前 episode
        - 'd': 丢弃当前未保存内容
    """
    data = request.get_json(silent=True) or {}
    arm_side = data.get('arm_side') if isinstance(data, dict) else None
    command = data.get('command', '') if isinstance(data, dict) else ''
    if arm_side not in RECORD_ARM_SIDES:
        return jsonify({"success": False, "message": "arm_side 必须为 left 或 right"}), 400
    process_name = recorder_process_name(arm_side)
    
    # 验证命令
    valid_commands = ['r', 's', 'd']
    if command not in valid_commands:
        return jsonify({
            'success': False,
            'message': f'Invalid command. Must be one of: {valid_commands}'
        })
    
    # 检查进程是否运行（使用全局active_processes）
    with process_lock:
        if process_name not in active_processes:
            return jsonify({
                'success': False,
                'message': 'Recording process is not running'
            })
        
        process = active_processes[process_name]
        
        # 检查进程是否还活着
        if process.poll() is not None:
            return jsonify({
                'success': False,
                'message': 'Recording process has stopped'
            })
        
        try:
            # 向进程的stdin发送命令
            if process.stdin is None:
                return jsonify({
                    'success': False,
                    'message': 'Recording process stdin is not available'
                })
            process.stdin.write(f'{command}\n')
            process.stdin.flush()
            
            command_desc = {
                'r': '切换开始/暂停 episode',
                's': '保存 episode（以 Recorder 日志确认为准）',
                'd': '丢弃未保存内容',
            }
            
            return jsonify({
                'success': True,
                'message': f'Command sent: {command_desc.get(command, command)}'
            })
        except Exception as e:
            return jsonify({
                'success': False,
                'message': f'Failed to send command: {str(e)}'
            })


@app.route('/api/ros/topics', methods=['GET'])
def get_ros_topics():
    """获取ROS话题列表"""
    try:
        setup_bash = PROJECT_ROOT / "install" / "setup.bash"
        result = subprocess.run(
            ['bash', '-c', f'source /opt/ros/humble/setup.bash && source {setup_bash} && ros2 topic list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        topics = result.stdout.strip().split('\n')
        return jsonify({
            "success": True,
            "topics": topics
        })
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


@app.route('/api/ros/nodes', methods=['GET'])
def get_ros_nodes():
    """获取ROS节点列表"""
    try:
        setup_bash = PROJECT_ROOT / "install" / "setup.bash"
        result = subprocess.run(
            ['bash', '-c', f'source /opt/ros/humble/setup.bash && source {setup_bash} && ros2 node list'],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        nodes = result.stdout.strip().split('\n')
        return jsonify({
            "success": True,
            "nodes": nodes
        })
    except Exception as e:
        return jsonify({"success": False, "message": str(e)})


# ==================== WebSocket 事件处理 ====================

@socketio.on('connect')
def handle_connect():
    """客户端连接"""
    logger.info("客户端已连接")
    emit('connection_response', {'status': 'connected'})


@socketio.on('disconnect')
def handle_disconnect():
    """客户端断开"""
    logger.info("客户端已断开")


@socketio.on('request_status')
def handle_status_request():
    """请求系统状态"""
    status = ProcessManager.get_status()
    emit('status_update', status)


# ==================== 主程序 ====================

if __name__ == '__main__':
    logger.info("🚀 启动 XArm 控制 Web UI 服务器")
    logger.info(f"📁 项目根目录: {PROJECT_ROOT}")
    logger.info(f"🎮 遥操作脚本: {TELEOP_SCRIPT_DIR}")
    logger.info(f"📹 数据采集: {COLLECTOR_DIR}")
    host = os.environ.get("XARM_UI_HOST", "0.0.0.0")
    port = int(os.environ.get("XARM_UI_PORT", "5000"))
    logger.info(f"🌐 访问地址: http://localhost:{port}")
    
    # 启动服务器
    socketio.run(app, host=host, port=port, debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
