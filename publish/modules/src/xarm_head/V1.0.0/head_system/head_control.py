#!/usr/bin/env python3
""" ROS2 + terminal position demo for the two-axis DM4310 head."""

import fcntl
import math
import os
import sys
import threading
import time

import rclpy
import k1_head_waist_can as xarm_can
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


CAN_INTERFACE = "can0"
PITCH_IDS = (0x0B, 0x1B)  # send, receive
YAW_IDS = (0x0C, 0x1C)
PITCH_LIMIT = (0.0, 24.0)
YAW_LIMIT = (-45.0, 45.0)
KP, KD = 40.0, 1.2
MAX_SPEED = 20.0  # deg/s
CONTROL_HZ = 500.0
RECV_TIMEOUT_US = 200
MAX_CYCLE_ERRORS = 5
LOCK_PATH = "/tmp/k1-can0-head-0b-0c.lock"


class HeadDriver:
    def __init__(self):
        self.xarm = None
        self.arm = None
        self.initial_position = None
        self.enabled = False
        self.lock_fd = None
        try:
            self.lock_fd = os.open(LOCK_PATH, os.O_CREAT | os.O_RDWR | os.O_CLOEXEC, 0o600)
            try:
                fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError as error:
                raise RuntimeError("another head controller owns can0 IDs 0x0B/0x0C") from error
            self.xarm = xarm_can.XArm(CAN_INTERFACE, True)
            self.xarm.init_arm_motors(
                [xarm_can.MotorType.DM4310] * 2,
                [PITCH_IDS[0], YAW_IDS[0]],
                [PITCH_IDS[1], YAW_IDS[1]],
            )
            self.arm = self.xarm.get_arm()
            self.xarm.set_callback_mode_all(xarm_can.CallbackMode.STATE)
            for _ in range(10):
                try:
                    self.initial_position = self.read(2000)
                    break
                except RuntimeError:
                    time.sleep(0.01)
            if self.initial_position is None:
                raise RuntimeError("no valid motor feedback while motors are disabled")
            error = HeadNode.feedback_error(*self.initial_position)
            if error:
                raise RuntimeError(f"unsafe initial motor feedback: {error}")
            self.write(*self.initial_position, kp=0.0, kd=0.0)
            self.enabled = True
            self.xarm.enable_all()
            self.write(*self.initial_position)
        except Exception:
            self.close()
            raise

    def read(self, timeout_us=RECV_TIMEOUT_US):
        self.xarm.refresh_all()
        self.xarm.recv_all(timeout_us)
        motors = self.arm.get_motors()
        if not all(m.get_state_tmos() or m.get_state_trotor() for m in motors):
            raise RuntimeError("motor feedback not received")
        return tuple(math.degrees(motor.get_position()) for motor in motors)

    def write(self, pitch_deg, yaw_deg, kp=KP, kd=KD):
        self.arm.mit_control_all(
            [
                xarm_can.MITParam(kp, kd, math.radians(pitch_deg), 0.0, 0.0),
                xarm_can.MITParam(kp, kd, math.radians(yaw_deg), 0.0, 0.0),
            ]
        )

    def close(self):
        try:
            if self.xarm is not None and self.enabled:
                self.xarm.disable_all()
                self.xarm.recv_all(1000)
        finally:
            self.enabled = False
            self.arm = None
            self.xarm = None
            if self.lock_fd is not None:
                os.close(self.lock_fd)
                self.lock_fd = None


def step_towards(current, target, step):
    delta = target - current
    return target if abs(delta) <= step else current + math.copysign(step, delta)


class HeadNode(Node):
    def __init__(self):
        super().__init__("head_control")
        self.lock = threading.Lock()
        self.target = None
        self.command = None
        self.feedback = None
        self.last_time = time.monotonic()
        self.cycle_errors = 0
        self.closed = False

        self.pub = self.create_publisher(Float64MultiArray, "/head/state", 10)
        self.sub = self.create_subscription(
            Float64MultiArray, "/head/cmd", self.ros_command, 10
        )
        self.driver = HeadDriver()
        try:
            error = self.feedback_error(*self.driver.initial_position)
            if error:
                raise RuntimeError(f"unsafe motor feedback: {error}")
            self.command = self.driver.initial_position
            self.target = self.command
            self.feedback = self.command
            self.driver.write(*self.command)
            self.timer = self.create_timer(1.0 / CONTROL_HZ, self.control_cycle)
        except Exception:
            self.driver.close()
            raise

        self.get_logger().info(
            f"Head ready at pitch={self.command[0]:.2f}, yaw={self.command[1]:.2f} deg; "
            "input [pitch_deg, yaw_deg], Ctrl+C to exit"
        )

    @staticmethod
    def target_error(pitch, yaw):
        if not math.isfinite(pitch) or not math.isfinite(yaw):
            return "angles must be finite numbers"
        if not PITCH_LIMIT[0] <= pitch <= PITCH_LIMIT[1]:
            return f"pitch must be in {PITCH_LIMIT} deg"
        if not YAW_LIMIT[0] <= yaw <= YAW_LIMIT[1]:
            return f"yaw must be in {YAW_LIMIT} deg"
        return None

    @staticmethod
    def feedback_error(pitch, yaw):
        if not math.isfinite(pitch) or not math.isfinite(yaw):
            return f"non-finite feedback: pitch={pitch}, yaw={yaw}"
        for name, angle, limits in (
            ("pitch", pitch, PITCH_LIMIT),
            ("yaw", yaw, YAW_LIMIT),
        ):
            if not limits[0] <= angle <= limits[1]:
                return f"{name} feedback {angle:.2f} deg is outside {limits} deg"
        return None

    def set_target(self, pitch, yaw, source="terminal"):
        error = self.target_error(pitch, yaw)
        if error:
            self.get_logger().warning(f"Rejected {source} command: {error}")
            return
        with self.lock:
            self.target = (pitch, yaw)
        self.get_logger().info(f"Target: pitch={pitch:.1f}, yaw={yaw:.1f} deg")

    def ros_command(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warning("Expected /head/cmd data: [pitch_deg, yaw_deg]")
            return
        self.set_target(float(msg.data[0]), float(msg.data[1]), "ROS")

    def print_feedback(self):
        with self.lock:
            feedback = self.feedback
        if feedback is None:
            print("Motor angle is not available yet")
        else:
            print(f"pitch={feedback[0]:.2f} deg, yaw={feedback[1]:.2f} deg")

    def control_cycle(self):
        try:
            pitch, yaw = self.driver.read()
            state = Float64MultiArray()
            state.data = [pitch, yaw]
            self.pub.publish(state)

            with self.lock:
                self.feedback = (pitch, yaw)
                target = self.target
            if target is None:
                self.cycle_errors = 0
                return

            if self.command is None:
                error = self.feedback_error(pitch, yaw)
                if error:
                    raise RuntimeError(f"unsafe motor feedback: {error}")
                self.command = (pitch, yaw)

            now = time.monotonic()
            step = MAX_SPEED * min(now - self.last_time, 0.1)
            self.last_time = now
            self.command = (
                step_towards(self.command[0], target[0], step),
                step_towards(self.command[1], target[1], step),
            )
            self.driver.write(*self.command)
            self.cycle_errors = 0
        except Exception as exc:
            self.cycle_errors += 1
            self.get_logger().error(
                f"Control error {self.cycle_errors}/{MAX_CYCLE_ERRORS}: {exc}"
            )
            if self.cycle_errors >= MAX_CYCLE_ERRORS:
                self.close()
                raise RuntimeError("head disabled after repeated control errors") from exc

    def close(self):
        if not self.closed:
            self.closed = True
            self.get_logger().info("Disabling head motors")
            self.driver.close()


def terminal_loop(node):
    print("Input: pitch_deg yaw_deg, or state. Ctrl+C exits.")
    while rclpy.ok():
        try:
            parts = input("head> ").replace(",", " ").split()
        except (EOFError, KeyboardInterrupt):
            return
        if len(parts) == 1 and parts[0].lower() == "state":
            node.print_feedback()
            continue
        if len(parts) != 2:
            print("Use: pitch_deg yaw_deg, or state")
            continue
        try:
            node.set_target(float(parts[0]), float(parts[1]))
        except ValueError:
            print("Pitch and yaw must be numbers")


def main():
    rclpy.init()
    node = None
    result = 0
    try:
        node = HeadNode()
        if sys.stdin.isatty():
            threading.Thread(target=terminal_loop, args=(node,), daemon=True).start()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        result = 1
        print(f"[ERROR] {exc}", file=sys.stderr)
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
