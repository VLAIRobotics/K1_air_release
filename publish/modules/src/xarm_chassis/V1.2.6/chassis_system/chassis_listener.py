#!/usr/bin/env python3
"""Receive-only K1 omni chassis listener for telemetry protocol V3.

This node has no joystick subscription and no CAN send path.  Legacy ``0x051``
is published only as ``/stm32/chassis_data`` compatibility data; V3 state and
target topics are built only from a fresh best-effort ``0x510``..``0x517``
snapshot produced by :mod:`chassis_telemetry_v3`.
"""

from __future__ import annotations

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

from chassis_telemetry_v3 import (
    ChassisTelemetryV3,
    ChassisTelemetryV3Assembler,
    TELEMETRY_V3_FLOAT64_FIELDS,
    TELEMETRY_V3_SNAPSHOT_CAN_ID,
    TELEMETRY_V3_START_CAN_ID,
    TELEMETRY_V3_STATUS_FLOAT64_FIELDS,
)


CAN_INTERFACE = "can2"
LEGACY_CAN_ID = 0x051
STATE_REQUIRED_FLAGS = 0x13  # wheel odometry, fused yaw, and four motors valid
STATUS_PERIOD_SEC = 0.2


def _layout(label_prefix: str, fields: tuple[str, ...]) -> list[MultiArrayDimension]:
    return [
        MultiArrayDimension(
            label=label_prefix + ":" + ",".join(fields),
            size=len(fields),
            stride=len(fields),
        )
    ]


class K1TelemetryListener(Node):
    """One receive-only SocketCAN node that publishes K1 V3 ROS views."""

    def __init__(self) -> None:
        super().__init__("k1_chassis_telemetry_listener")
        self._stop_event = threading.Event()
        self._lifecycle_lock = threading.RLock()
        # The CAN receive thread mutates the assembler while the ROS timer
        # reads its diagnostic snapshot.  Keep those compound operations
        # atomic so status never combines two different telemetry updates.
        self._assembler_lock = threading.RLock()
        self._closed = False
        self.bus = None
        self.recv_thread = None
        self.status_timer = None
        self.context.on_shutdown(self._on_ros_shutdown)
        self.assembler = ChassisTelemetryV3Assembler()

        try:
            self.bus = can.interface.Bus(channel=CAN_INTERFACE, interface="socketcan")
        except Exception as error:
            self.get_logger().error(f"Cannot open {CAN_INTERFACE}: {error}")
            raise

        self.legacy_pub = self.create_publisher(Float64MultiArray, "/stm32/chassis_data", 10)
        self.telemetry_pub = self.create_publisher(Float64MultiArray, "/chassis/telemetry_v3", 50)
        self.state_pub = self.create_publisher(Odometry, "/chassis/state", 50)
        self.target_pub = self.create_publisher(TwistStamped, "/chassis/target_velocity", 50)
        self.status_pub = self.create_publisher(Float64MultiArray, "/chassis/telemetry_v3_status", 10)
        self.status_timer = self.create_timer(STATUS_PERIOD_SEC, self._publish_status)
        self.recv_thread = threading.Thread(target=self._recv_loop, name="k1-can2-rx", daemon=False)
        self.recv_thread.start()
        self.get_logger().info("K1 V1.2.6 listener started: receive-only can2 V3 telemetry")

    def _on_ros_shutdown(self) -> None:
        self._stop_event.set()

    def _should_run(self) -> bool:
        return not self._stop_event.is_set() and self.context.ok()

    def _safe_publish(self, publisher, message, topic: str) -> bool:
        if not self._should_run():
            return False
        try:
            publisher.publish(message)
            return True
        except RCLError as error:
            if not self._should_run():
                return False
            self.get_logger().error(f"ROS publish failed for {topic}: {error}")
            raise

    @staticmethod
    def _set_stamp(message, sample: ChassisTelemetryV3) -> None:
        sec, nanosec = sample.host_receive_sec_nanosec
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = nanosec
        message.header.frame_id = "chassis_odom"

    def _recv_loop(self) -> None:
        while self._should_run():
            with self._lifecycle_lock:
                bus = self.bus
            if bus is None:
                return
            try:
                message = bus.recv(timeout=1.0)
            except can.CanError as error:
                if self._should_run():
                    self.get_logger().error(f"CAN receive error: {error}")
                return
            if not self._should_run():
                return
            if message is None:
                continue
            received_monotonic = time.monotonic()
            received_wall_ns = time.time_ns()
            can_id = message.arbitration_id
            if can_id == LEGACY_CAN_ID:
                self._publish_legacy_051(message.data, message.dlc)
                continue
            if not TELEMETRY_V3_START_CAN_ID <= can_id <= TELEMETRY_V3_SNAPSHOT_CAN_ID:
                continue
            with self._assembler_lock:
                sample = self.assembler.ingest(
                    can_id,
                    message.data,
                    dlc=message.dlc,
                    is_extended_id=message.is_extended_id,
                    is_error_frame=message.is_error_frame,
                    host_receive_monotonic=received_monotonic,
                    host_receive_wall_time_ns=received_wall_ns,
                )
            if sample is not None and self._should_run():
                self._publish_snapshot(sample)

    def _publish_legacy_051(self, data: bytes, dlc: int) -> None:
        """Keep V1.2.x big-endian legacy data isolated from all K1 V3 state."""

        if dlc != 8 or len(data) != 8:
            self.get_logger().warn("Ignoring malformed legacy 0x051 frame")
            return
        x_raw, y_raw, yaw_raw = struct.unpack(">hhh", bytes(data[:6]))
        message = Float64MultiArray()
        message.data = [
            x_raw / 1000.0 * 0.5,
            y_raw / 1000.0 * 0.5,
            math.degrees(yaw_raw / 1000.0),
            float(data[6] & 0x0F),
            float((data[7] >> 4) & 0x0F),
            float(data[7] & 0x0F),
        ]
        self._safe_publish(self.legacy_pub, message, "/stm32/chassis_data")

    def _publish_snapshot(self, sample: ChassisTelemetryV3) -> None:
        telemetry = Float64MultiArray()
        telemetry.layout.dim = _layout("chassis_telemetry_v3", TELEMETRY_V3_FLOAT64_FIELDS)
        telemetry.data = sample.to_float64_values()
        if not self._safe_publish(self.telemetry_pub, telemetry, "/chassis/telemetry_v3"):
            return

        target = TwistStamped()
        self._set_stamp(target, sample)
        target.twist.linear.x = sample.target_vx_m_s
        target.twist.linear.y = sample.target_vy_m_s
        target.twist.linear.z = 0.0
        target.twist.angular.x = 0.0
        target.twist.angular.y = 0.0
        target.twist.angular.z = sample.target_wz_rad_s
        if not self._safe_publish(self.target_pub, target, "/chassis/target_velocity"):
            return

        if (sample.valid_flags & STATE_REQUIRED_FLAGS) != STATE_REQUIRED_FLAGS:
            self.get_logger().warn(
                f"Skipping /chassis/state for seq={sample.seq}: valid_flags=0x{sample.valid_flags:02X}"
            )
            return
        state = Odometry()
        self._set_stamp(state, sample)
        state.child_frame_id = "chassis_base_link"
        state.pose.pose.position.x = sample.x_m
        state.pose.pose.position.y = sample.y_m
        state.pose.pose.position.z = 0.0
        state.pose.pose.orientation.z = math.sin(sample.fused_yaw_rad / 2.0)
        state.pose.pose.orientation.w = math.cos(sample.fused_yaw_rad / 2.0)
        state.twist.twist.linear.x = sample.actual_vx_m_s
        state.twist.twist.linear.y = sample.actual_vy_m_s
        state.twist.twist.linear.z = 0.0
        state.twist.twist.angular.x = 0.0
        state.twist.twist.angular.y = 0.0
        state.twist.twist.angular.z = sample.actual_wz_rad_s
        self._safe_publish(self.state_pub, state, "/chassis/state")

    def _publish_status(self) -> None:
        if not self._should_run():
            return
        with self._assembler_lock:
            status = self.assembler.status()
        message = Float64MultiArray()
        message.layout.dim = _layout("chassis_telemetry_v3_status", TELEMETRY_V3_STATUS_FLOAT64_FIELDS)
        message.data = status.to_float64_values()
        self._safe_publish(self.status_pub, message, "/chassis/telemetry_v3_status")

    def close(self) -> None:
        """Stop the receive thread and release SocketCAN exactly once."""

        with self._lifecycle_lock:
            if self._closed:
                return
            self._closed = True
            self._stop_event.set()
            timer, self.status_timer = self.status_timer, None
            bus, self.bus = self.bus, None
            receiver = self.recv_thread
        if timer is not None:
            try:
                timer.cancel()
            except RCLError:
                pass
        if bus is not None:
            try:
                bus.shutdown()
            except can.CanError:
                pass
        if receiver is not None and receiver is not threading.current_thread():
            receiver.join(timeout=2.0)
            if receiver.is_alive():
                raise RuntimeError("K1 CAN receiver did not exit during close()")


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = K1TelemetryListener()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
