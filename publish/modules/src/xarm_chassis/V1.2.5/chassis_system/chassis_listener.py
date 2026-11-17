#!/usr/bin/env python3
"""L1 Air V1.2.5 chassis listener.

Compatibility input ``0x051`` remains available for the existing white remote
and ``/stm32/chassis_data`` consumers. Physical odometry, actual velocity and
target velocity are published only after a complete, valid ``0x510`` through
``0x514`` telemetry group is committed.
"""

import math
import struct
import threading
import time

import can
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy._rclpy_pybind11 import RCLError
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from chassis_telemetry_v1 import (
    ChassisTelemetryAssembler,
    ChassisTelemetryV1,
    TELEMETRY_COMMIT_CAN_ID,
    TELEMETRY_START_CAN_ID,
    TELEMETRY_V1_FLOAT64_FIELDS,
)


class JoystickToCan(Node):
    """Receive chassis telemetry and retain the V1.2.2 joystick compatibility path."""

    def __init__(self):
        super().__init__("joystick_to_can")

        # V1.2.2 compatibility settings. The telemetry receiver never sends;
        # this existing timer only sends when a valid joystick message arrived.
        self.max_speed = 800
        self.can_id = 0x050
        self.send_hz = 200.0
        self.latest_joy = None
        self.joy_lock = threading.Lock()
        self.updown_data = 0

        # Keep shutdown ownership in this node.  The context callback must stay
        # fast because it may run from rclpy signal-handling code.
        self._can_stop_event = threading.Event()
        self._lifecycle_lock = threading.RLock()
        self._closed = False
        self.bus = None
        self.recv_thread = None
        self.timer = None
        self.context.on_shutdown(self._on_ros_shutdown)

        # 0x051 legacy conversion. Do not use these values for the new state.
        self.ODOM_LINEAR_SCALE = 1.0 / 1000.0
        self.ODOM_ANGULAR_SCALE = 1.0 / 1000.0
        self.ODOM_LINEAR_CORRECTION = 0.5
        self.telemetry_assembler = ChassisTelemetryAssembler()

        try:
            self.bus = can.interface.Bus(channel="can2", interface="socketcan")
            self.get_logger().info("CAN bus can2 opened")
        except Exception as error:
            self.get_logger().error(f"CAN init failed: {error}")
            raise

        self.sub = self.create_subscription(
            Float64MultiArray, "/dual_xarm/joystick", self.joystick_callback, 10
        )
        self.subdate = self.create_subscription(
            Float64MultiArray,
            "/dual_xarm/py_motor_target",
            self.py_motor_position_callback,
            10,
        )

        # 0x051 legacy topic. Its old array order remains
        # [x_m, y_m, yaw_deg, ch4, ch5, ch6].
        self.chassis_pub = self.create_publisher(
            Float64MultiArray, "/stm32/chassis_data", 10
        )

        # No generated custom message package exists in this workspace. Until
        # one is added, this is a documented float64-array integration topic;
        # TELEMETRY_V1_FLOAT64_FIELDS defines the exact field order.
        self.telemetry_pub = self.create_publisher(
            Float64MultiArray, "/chassis/telemetry_v1", 50
        )
        self.chassis_state_pub = self.create_publisher(Odometry, "/chassis/state", 50)
        self.target_velocity_pub = self.create_publisher(
            TwistStamped, "/chassis/target_velocity", 50
        )

        self.timer = self.create_timer(1.0 / self.send_hz, self.can_send_loop)
        self.recv_thread = threading.Thread(target=self.can_recv_loop, daemon=False)
        self.recv_thread.start()
        self.get_logger().info(
            "V1.2.5 listener started: 0x051 legacy compatibility and physical telemetry"
        )

    def joystick_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) != 4:
            return
        if not all(isinstance(value, float) and math.isfinite(value) for value in msg.data):
            return
        with self.joy_lock:
            self.latest_joy = msg.data[:]

    def _on_ros_shutdown(self) -> None:
        """Fast context callback: wake the owned receiver toward shutdown."""
        self._can_stop_event.set()

    def _receiver_should_run(self) -> bool:
        return not self._can_stop_event.is_set() and self.context.ok()

    def py_motor_position_callback(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 3:
            self.get_logger().warn("PY motor msg length < 3")
            return
        self.updown_data = float(data[2])

    def can_send_loop(self) -> None:
        """Unchanged V1.2.2 joystick sender; no joystick means no 0x050 send."""
        if not self._receiver_should_run():
            return
        with self.joy_lock:
            if self.latest_joy is None:
                return
            joy = self.latest_joy
        with self._lifecycle_lock:
            bus = self.bus
        if bus is None:
            return

        x = joy[0]
        y = joy[1]
        vx = max(-self.max_speed, min(self.max_speed, int(x * self.max_speed)))
        vy = max(-self.max_speed, min(self.max_speed, int(y * self.max_speed)))
        vw = 0
        if self.updown_data == 1:
            updown = 2
        elif self.updown_data == -1:
            updown = 1
        else:
            updown = 0

        data = bytearray(8)
        data[0:2] = struct.pack("<h", vx)
        data[2:4] = struct.pack("<h", vy)
        data[4:6] = struct.pack("<h", vw)
        data[6] = 0
        data[7] = updown
        try:
            bus.send(
                can.Message(arbitration_id=self.can_id, data=data, is_extended_id=False)
            )
        except can.CanError:
            return

    def can_recv_loop(self) -> None:
        self.get_logger().info("CAN receive thread started")
        while self._receiver_should_run():
            with self._lifecycle_lock:
                bus = self.bus
            if bus is None:
                return
            try:
                message = bus.recv(timeout=1.0)
                if not self._receiver_should_run():
                    return
                if message is None:
                    continue
                receive_monotonic = time.monotonic()
                receive_wall_time_ns = time.time_ns()
                can_id = message.arbitration_id

                if can_id == 0x051:
                    if not self._handle_legacy_0x051(message.data, message.dlc):
                        return
                elif TELEMETRY_START_CAN_ID <= can_id <= TELEMETRY_COMMIT_CAN_ID:
                    sample = self.telemetry_assembler.ingest(
                        can_id,
                        message.data,
                        dlc=message.dlc,
                        is_extended_id=message.is_extended_id,
                        is_error_frame=message.is_error_frame,
                        host_receive_monotonic=receive_monotonic,
                        host_receive_wall_time_ns=receive_wall_time_ns,
                    )
                    if sample is not None:
                        if not self._receiver_should_run():
                            return
                        if not self._publish_physical_telemetry(sample):
                            return
            except can.CanError as error:
                if not self._receiver_should_run():
                    return
                self.get_logger().error(f"CAN recv failed: {error}")

    def _safe_publish(self, publisher, message, topic_name: str) -> bool:
        """Publish one message without hiding an RCLError from a live context."""
        if not self._receiver_should_run():
            return False
        try:
            publisher.publish(message)
        except RCLError as error:
            if not self._receiver_should_run():
                return False
            self.get_logger().error(f"ROS publish failed for {topic_name}: {error}")
            raise
        return self._receiver_should_run()

    def _handle_legacy_0x051(self, data: bytes, dlc: int) -> bool:
        """Keep the old 0x051 display/remote compatibility topic unchanged."""
        if dlc != 8 or len(data) != 8:
            self.get_logger().warn("Ignoring malformed legacy 0x051 frame")
            return True
        odom_x_raw = struct.unpack(">h", data[0:2])[0]
        odom_y_raw = struct.unpack(">h", data[2:4])[0]
        odom_z_raw = struct.unpack(">h", data[4:6])[0]
        ch4_state = int(data[6] & 0x0F)
        ch5_state = int((data[7] >> 4) & 0x0F)
        ch6_state = int(data[7] & 0x0F)

        legacy_msg = Float64MultiArray()
        legacy_msg.data = [
            float(odom_x_raw * self.ODOM_LINEAR_SCALE * self.ODOM_LINEAR_CORRECTION),
            float(odom_y_raw * self.ODOM_LINEAR_SCALE * self.ODOM_LINEAR_CORRECTION),
            float(math.degrees(odom_z_raw * self.ODOM_ANGULAR_SCALE)),
            float(ch4_state),
            float(ch5_state),
            float(ch6_state),
        ]
        return self._safe_publish(self.chassis_pub, legacy_msg, "/stm32/chassis_data")

    @staticmethod
    def _set_header_stamp(message, sample: ChassisTelemetryV1) -> None:
        sec, nanosec = sample.host_receive_sec_nanosec
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = nanosec
        message.header.frame_id = "chassis_odom"

    def _publish_physical_telemetry(self, sample: ChassisTelemetryV1) -> bool:
        """Publish every output from one 0x510..0x514 sample with one stamp."""
        telemetry_msg = Float64MultiArray()
        telemetry_msg.layout.dim = [
            MultiArrayDimension(
                label="chassis_telemetry_v1:" + ",".join(TELEMETRY_V1_FLOAT64_FIELDS),
                size=len(TELEMETRY_V1_FLOAT64_FIELDS),
                stride=len(TELEMETRY_V1_FLOAT64_FIELDS),
            )
        ]
        telemetry_msg.data = sample.to_float64_values()
        if not self._safe_publish(
            self.telemetry_pub, telemetry_msg, "/chassis/telemetry_v1"
        ):
            return False

        odometry = Odometry()
        self._set_header_stamp(odometry, sample)
        odometry.child_frame_id = "chassis_base_link"
        odometry.pose.pose.position.x = sample.x_m
        odometry.pose.pose.position.y = sample.y_m
        # A quaternion necessarily wraps orientation, while telemetry_v1 keeps
        # yaw_total_rad continuous for Recorder use.
        odometry.pose.pose.orientation.z = math.sin(sample.yaw_total_rad / 2.0)
        odometry.pose.pose.orientation.w = math.cos(sample.yaw_total_rad / 2.0)
        odometry.twist.twist.linear.x = sample.actual_vx_m_s
        odometry.twist.twist.linear.y = 0.0
        odometry.twist.twist.angular.z = sample.actual_wz_rad_s
        if not self._safe_publish(self.chassis_state_pub, odometry, "/chassis/state"):
            return False

        target = TwistStamped()
        self._set_header_stamp(target, sample)
        target.twist.linear.x = sample.target_vx_m_s
        target.twist.linear.y = 0.0
        target.twist.angular.z = sample.target_wz_rad_s
        return self._safe_publish(
            self.target_velocity_pub, target, "/chassis/target_velocity"
        )

    def close(self) -> None:
        """Close timer, SocketCAN and receiver exactly once before node destroy."""
        with self._lifecycle_lock:
            if self._closed:
                return
            self._closed = True
            self._can_stop_event.set()
            timer = self.timer
            self.timer = None
            bus = self.bus
            self.bus = None
            recv_thread = self.recv_thread

        if timer is not None:
            try:
                timer.cancel()
            except RCLError:
                # The ROS context may already have been shut down by SIGINT.
                pass
        if bus is not None:
            try:
                bus.shutdown()
            except can.CanError:
                # Expected when shutdown is also waking a blocking recv().
                pass
        if recv_thread is not None and recv_thread is not threading.current_thread():
            recv_thread.join(timeout=2.0)
            if recv_thread.is_alive():
                raise RuntimeError("CAN receiver did not exit during close()")


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = JoystickToCan()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except RCLError:
                if rclpy.ok():
                    raise


if __name__ == "__main__":
    main()
