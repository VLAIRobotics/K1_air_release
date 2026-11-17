#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
import can
import struct
import threading
import math


# Verified 0x051 conversions. Keep one source of truth for both ROS topics.
ODOM_LINEAR_SCALE = 0.001
ODOM_LINEAR_CORRECTION = 0.5
ODOM_ANGULAR_SCALE = 0.001


def decode_odom_051(data, dlc):
    """Decode one complete 0x051 payload using the verified MCU protocol.

    Returns None for malformed frames.  The returned ``corrected_x/y`` values
    intentionally preserve the established 0.5 odometry correction.
    """
    if dlc != 8 or len(data) != 8:
        return None

    raw_x, raw_y, raw_yaw = struct.unpack('>hhh', bytes(data[:6]))
    mcu_x = raw_x * ODOM_LINEAR_SCALE
    mcu_y = raw_y * ODOM_LINEAR_SCALE
    yaw_rad = raw_yaw * ODOM_ANGULAR_SCALE
    return {
        'raw_x': raw_x,
        'raw_y': raw_y,
        'raw_yaw': raw_yaw,
        'corrected_x': mcu_x * ODOM_LINEAR_CORRECTION,
        'corrected_y': mcu_y * ODOM_LINEAR_CORRECTION,
        'yaw_rad': yaw_rad,
        'yaw_deg': math.degrees(yaw_rad),
        'ch4': int(data[6] & 0x0F),
        'ch5': int((data[7] >> 4) & 0x0F),
        'ch6': int(data[7] & 0x0F),
    }


class JoystickToCan(Node):

    def __init__(self):
        super().__init__('joystick_to_can')

        # ================= 参数 =================
        self.max_speed = 800
        self.can_id = 0x050
        self.send_hz = 200.0  # CAN 发送频率（Hz）
        self.declare_parameter('can_channel', 'can3')
        self.can_channel = str(self.get_parameter('can_channel').value)
        if not self.can_channel:
            raise ValueError('can_channel must not be empty')

        # ================= 状态缓存 =================
        self.latest_joy = None
        self.joy_lock = threading.Lock()

        self.updown_data = 0
        self._invalid_051_count = 0
        self._can_shutdown_event = threading.Event()
        self._lifecycle_lock = threading.Lock()
        self._destroyed = False
        self.recv_thread = None
        self.bus = None

        # ================= CAN 初始化 =================
        try:
            self.bus = can.interface.Bus(
                channel=self.can_channel,
                interface='socketcan'
            )
            self.get_logger().info(f'CAN bus {self.can_channel} opened')
        except Exception as e:
            self.get_logger().error(f'CAN init failed: {e}')
            raise
        # ================= ROS 订阅 =================
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/dual_xarm/joystick',
            self.joystick_callback,
            10
        )
        self.subdate = self.create_subscription(
            Float64MultiArray,
            '/dual_xarm/py_motor_target',
            self.py_motor_position_callback,
            10
        )

        # ================= ROS 发布 =================
        # 发布底盘累积位移（横向位移x, 前后位移y, 旋转角度w）
        self.chassis_pub = self.create_publisher(
            Float64MultiArray,
            '/stm32/chassis_data',
            10
        )
        self.chassis_state_pub = self.create_publisher(
            Odometry,
            '/chassis/state',
            10
        )

        # ================= 定时器（CAN 发送） =================
        self.timer = self.create_timer(
            1.0 / self.send_hz,
            self.can_send_loop
        )

        # Context shutdown may precede main() reaching destroy_node().  This
        # callback must stay non-blocking; destroy_node() owns resource release.
        self.context.on_shutdown(self._on_ros_shutdown)

        # Start asynchronous CAN reception only after every publisher,
        # subscriber, shared state field, lock, and timer it can interact with
        # has been constructed.
        self.recv_thread = threading.Thread(
             target=self.can_recv_loop,
             daemon=False
        )
        self.recv_thread.start()

        self.get_logger().info('Joystick → CAN node started')

    # ==================================================
    #                Joystick Callback
    # ==================================================
    def joystick_callback(self, msg: Float64MultiArray):
        # 只做数据接收 & 校验

        if len(msg.data) != 4:
            return

        # 数据合法性检查
        for v in msg.data:
            if not isinstance(v, float) or not math.isfinite(v):
                return

        with self.joy_lock:
            self.latest_joy = msg.data[:]  # 拷贝一份

    def py_motor_position_callback(self, msg):
        data = list(msg.data)
        if len(data) < 3:
            # 消息长度不足：数据包损坏或发布端格式错误，跳过本帧
            self.get_logger().warn("⚠️ PY motor msg length < 2")
            return
        self.updown_data = float(data[2])
    # ==================================================
    #                CAN Send Loop
    # ==================================================
    def can_send_loop(self):
        with self.joy_lock:
            if self.latest_joy is None:
                return
            joy = self.latest_joy

        # ================= 解析数据 =================
        x    = joy[0]   # 左右
        y    = joy[1]   # 前后
        down = self.updown_data  # 上下

        # ================= 映射速度 =================
        # vx = 0
        # vy = int(-y * self.max_speed)
        # vw = int(x * self.max_speed)

        # vy = max(-self.max_speed, min(self.max_speed, vy))
        # vw = max(-self.max_speed, min(self.max_speed, vw))

        vx = int(x * self.max_speed)
        vy = int(y * self.max_speed)
        vw = 0

        vy = max(-self.max_speed, min(self.max_speed, vy))
        vx = max(-self.max_speed, min(self.max_speed, vx))



        if self.updown_data == 1:
            updown = 2
        elif self.updown_data == -1:
            updown = 1
        else:
            updown = 0
        print(f"Joystick: x={x:.2f}, y={y:.2f}, down={self.updown_data} → vx={vx}, vy={vy}, vw={vw}, updown={updown}")
        # ================= 打包 CAN =================
        data = bytearray(8)
        data[0:2] = struct.pack('<h', vx)
        data[2:4] = struct.pack('<h', vy)
        data[4:6] = struct.pack('<h', vw)
        data[6] = 0
        data[7] = updown

        # ================= 发送 CAN =================
        try:
            msg = can.Message(
                arbitration_id=self.can_id,
                data=data,
                is_extended_id=False
            )
            self.bus.send(msg)
        except can.CanError:
            return

        self.get_logger().debug(
            f'CAN sent: vx={vx}, vy={vy}, vw={vw}, updown={updown}'
        )

    def publish_chassis_state(self, decoded):
        """Publish reliable 0x051 pose only; MCU does not provide twist feedback."""
        if self._should_stop_can():
            return False

        yaw_rad = decoded['yaw_rad']
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        norm = math.hypot(qz, qw)
        if norm == 0.0:
            self.get_logger().error('Refusing to publish chassis state: zero-norm yaw quaternion')
            return False

        state_msg = Odometry()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = 'chassis_odom'
        state_msg.child_frame_id = 'chassis_base_link'
        state_msg.pose.pose.position.x = decoded['corrected_x']
        state_msg.pose.pose.position.y = decoded['corrected_y']
        state_msg.pose.pose.position.z = 0.0
        state_msg.pose.pose.orientation.x = 0.0
        state_msg.pose.pose.orientation.y = 0.0
        state_msg.pose.pose.orientation.z = qz / norm
        state_msg.pose.pose.orientation.w = qw / norm
        return self._publish_from_can_thread(self.chassis_state_pub, state_msg)

    def _on_ros_shutdown(self):
        """Fast context callback: tell the CAN thread to stop publishing."""
        self._can_shutdown_event.set()

    def _should_stop_can(self):
        return self._can_shutdown_event.is_set() or not self.context.ok()

    def _publish_from_can_thread(self, publisher, message):
        """Publish one 0x051-derived message, stopping cleanly on shutdown."""
        if self._should_stop_can():
            return False

        try:
            publisher.publish(message)
        except RCLError:
            if self._should_stop_can():
                return False
            self.get_logger().error('ROS publish failed while context remained valid')
            raise
        return True

    def _shutdown_can_bus(self):
        """Close the node-owned SocketCAN bus exactly once."""
        with self._lifecycle_lock:
            bus = self.bus
            self.bus = None

        if bus is None:
            return

        try:
            bus.shutdown()
        except (can.CanError, OSError) as error:
            self.get_logger().error(f'CAN bus shutdown failed: {error}')

    def destroy_node(self):
        """Stop node-owned CAN resources before destroying ROS entities."""
        with self._lifecycle_lock:
            if self._destroyed:
                return
            self._destroyed = True
            self._can_shutdown_event.set()
            recv_thread = self.recv_thread
            timer = self.timer

        timer.cancel()

        if recv_thread is not threading.current_thread() and recv_thread.is_alive():
            recv_thread.join(timeout=2.0)

        if recv_thread.is_alive():
            # A compliant SocketCAN receive unblocks within its finite timeout.
            # Closing the node-owned bus is the bounded fallback for a blocked recv.
            self._shutdown_can_bus()
            if recv_thread is not threading.current_thread():
                recv_thread.join(timeout=2.0)
        else:
            self._shutdown_can_bus()

        if recv_thread.is_alive():
            self.get_logger().error('CAN receive thread did not exit during node shutdown')

        super().destroy_node()

# ==================================================
#                CAN Receive Loop
# ==================================================
    def can_recv_loop(self):

        self.get_logger().info("CAN receive thread started")

        while not self._should_stop_can():

            try:
                bus = self.bus
                if bus is None:
                    break
                msg = bus.recv(timeout=1.0)

                if self._should_stop_can():
                    break

                if msg is None:
                    continue

                # ================= 基本信息 =================
                can_id = msg.arbitration_id
                data = msg.data
                dlc = msg.dlc

                # print(f"\n[CAN RX]")
                # print(f"ID   : 0x{can_id:03X}")
                # print(f"DLC  : {dlc}")
                # print(f"DATA : {[hex(b) for b in data]}")

                # ================= 里程计解析 =================
                # STM32 发送格式：
                # [0:2] x, [2:4] y, [4:6] z
                # [6] ch4_state(0/1/2)
                # [7] 高4位 ch5_state(0/1), 低4位 ch6_state(0/1)
                # x/y/z 均为大端有符号 int16，且数值 = 实际值 * 1000
                if can_id == 0x051:
                    decoded = decode_odom_051(data, dlc)
                    if decoded is None:
                        self._invalid_051_count += 1
                        if self._invalid_051_count == 1 or self._invalid_051_count % 100 == 0:
                            self.get_logger().error(
                                f'Ignoring invalid 0x051 frame: dlc={dlc}, bytes={len(data)}, '
                                f'count={self._invalid_051_count}'
                            )
                        continue

                    if self._should_stop_can():
                        break

                    odom_x_raw = decoded['raw_x']
                    odom_y_raw = decoded['raw_y']
                    odom_z_raw = decoded['raw_yaw']
                    ch4_state = decoded['ch4']
                    ch5_state = decoded['ch5']
                    ch6_state = decoded['ch6']

                    # 直接还原为 STM32 当前发送的物理量：
                    # x/y 单位 m，z 原始单位 rad
                    vx_display = decoded['corrected_x']
                    vy_display = decoded['corrected_y']
                    vw_display = decoded['yaw_deg']

                    # ================= 发布显示数据 =================
                    chassis_msg = Float64MultiArray()
                    chassis_msg.data = [
                        float(vx_display),
                        float(vy_display),
                        float(vw_display),
                        float(ch4_state),
                        float(ch5_state),
                        float(ch6_state),
                    ]
                    if not self._publish_from_can_thread(self.chassis_pub, chassis_msg):
                        break
                    if not self.publish_chassis_state(decoded):
                        break

                    # ================= 日志输出（三个数据都显示） =================
                    self.get_logger().info(
                        f"Chassis: display(x={vx_display:.4f}m, y={vy_display:.4f}m, w={vw_display:.2f}deg, ch4={ch4_state}, ch5={ch5_state}, ch6={ch6_state}) | raw(x={odom_x_raw}, y={odom_y_raw}, z={odom_z_raw})"
                    )

            except (can.CanError, OSError) as error:
                if self._can_shutdown_event.is_set():
                    break
                self.get_logger().error(f"CAN recv failed: {error}")


def main():
    rclpy.init()
    node = None
    try:
        node = JoystickToCan()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
