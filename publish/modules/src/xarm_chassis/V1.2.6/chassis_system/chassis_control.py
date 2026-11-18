#!/usr/bin/env python3
"""L1-compatible ROS String and terminal control for the K1 omni chassis."""

from __future__ import annotations

import struct
import sys
import threading
from dataclasses import dataclass

import chassis_omni_velocity_command as can050


SEND_RATE_HZ = 30.0
MAX_RAW_SPEED = can050.RAW_LIMIT
DEFAULT_SPEED_RAW = 306

# Keep the L1 command names: left/right are turns, not lateral moves.
# K1 uses vx for forward and a negative raw vw for a left turn.
# Tuples are raw (vx, vy, vw); legacy CAN Y is right-positive.
CHASSIS_DIRECTIONS = {
    "stop": (0, 0, 0),
    "forward": (1, 0, 0),
    "backward": (-1, 0, 0),
    "left": (0, 0, -1),
    "right": (0, 0, 1),
    "strafe_left": (0, -1, 0),
    "strafe_right": (0, 1, 0),
}
# K1 onmi_car controler_task.c: byte 7 is 1 up, 2 down (unlike L1).
LIFT_STATES = {"lift_stop": 0, "lift_up": 1, "lift_down": 2}
MENU_COMMANDS = {
    "1": "stop",
    "2": "left",
    "3": "right",
    "4": "forward",
    "5": "backward",
    "6": "strafe_left",
    "7": "strafe_right",
    "8": "lift_up",
    "9": "lift_down",
    "0": "lift_stop",
}


@dataclass
class ChassisController:
    speed_raw: int = DEFAULT_SPEED_RAW
    chassis_state: str = "stop"
    lift_state: str = "lift_stop"

    def apply_legacy_command(self, command: str) -> bool:
        command = command.strip().lower()
        if command in CHASSIS_DIRECTIONS:
            self.chassis_state = command
            return True
        if command in LIFT_STATES:
            self.lift_state = command
            return True
        if command.startswith("speed_"):
            raw = command.removeprefix("speed_")
            if raw.isascii() and raw.isdecimal():
                raw = raw.lstrip("0") or "0"
                if len(raw) <= len(str(MAX_RAW_SPEED)):
                    speed = int(raw)
                    if speed <= MAX_RAW_SPEED:
                        self.speed_raw = speed
                        return True
        return False

    def current_payload(self) -> bytes:
        vx_multiplier, vy_multiplier, vw_multiplier = CHASSIS_DIRECTIONS[self.chassis_state]
        speed = self.speed_raw
        return struct.pack("<hhhBB", vx_multiplier * speed, vy_multiplier * speed, vw_multiplier * speed,
                           0, LIFT_STATES[self.lift_state])


def terminal_input_loop(controller: ChassisController, exit_event: threading.Event) -> None:
    print("===== K1 V1.2.6 底盘控制 =====")
    print("1=停止  2=左转  3=右转  4=前进  5=后退")
    print("6=左平移  7=右平移")
    print("8=立柱上升  9=立柱下降  0=立柱停止  s=输入 raw 速度  q=停止并退出")
    while not exit_event.is_set():
        try:
            command = input(f"请输入命令 [0-9/s/q] (raw速度:{controller.speed_raw}): ").strip()
            if command == "s":
                raw = input(f"输入 raw 速度 [0-{MAX_RAW_SPEED}]: ").strip()
                mapped = f"speed_{raw}"
            else:
                mapped = MENU_COMMANDS.get(command)
        except (EOFError, KeyboardInterrupt):
            command = "q"
        if command == "q":
            exit_event.set()
            return
        if mapped is not None and controller.apply_legacy_command(mapped):
            print(f"底盘={controller.chassis_state}, raw速度={controller.speed_raw}, 立柱={controller.lift_state}")
        else:
            print("无效命令")


def main() -> int:
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as error:
        print(f"error: K1 chassis_control.py requires ROS 2 Humble: {error}", file=sys.stderr)
        return 2

    controller = ChassisController()
    exit_event = threading.Event()

    class ChassisControlROS2(Node):
        def __init__(self, bus, can_module):
            super().__init__("chassis_control_ros2")
            self.sub = self.create_subscription(String, "/chassis/cmd", self.on_command, 10)
            self.timer = self.create_timer(1.0 / SEND_RATE_HZ, self.send_current)
            self.bus = bus
            self.can_module = can_module

        def on_command(self, msg: String) -> None:
            if controller.apply_legacy_command(msg.data):
                self.get_logger().info(f"/chassis/cmd={msg.data.strip().lower()}; raw={controller.speed_raw}")
            else:
                self.get_logger().warn(f"unsupported /chassis/cmd: {msg.data!r}")

        def send_current(self) -> None:
            if not exit_event.is_set():
                can050._send_payload(self.bus, self.can_module, controller.current_payload())

    try:
        conflicts = can050.find_known_sender_processes()
        if conflicts:
            raise can050.OmniVelocityCommandError(
                "refusing CAN 0x050 transmission because known senders are running:\n" + "\n".join(conflicts)
            )
        with can050.AdvisoryLock():
            rclpy.init()
            bus = node = None
            try:
                bus, can_module = can050._open_can_bus()
                node = ChassisControlROS2(bus, can_module)
                node.get_logger().info("K1 can2/0x050 control: /chassis/cmd at 30 Hz")
                threading.Thread(target=terminal_input_loop, args=(controller, exit_event), daemon=True).start()
                while rclpy.ok() and not exit_event.is_set():
                    rclpy.spin_once(node, timeout_sec=0.1)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                try:
                    if bus is not None:
                        try:
                            can050.send_zero_sequence(bus, can_module, SEND_RATE_HZ)
                        finally:
                            bus.shutdown()
                finally:
                    if node is not None:
                        node.destroy_node()
                    if rclpy.ok():
                        rclpy.shutdown()
    except Exception as error:
        print(f"error: K1 chassis control failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
