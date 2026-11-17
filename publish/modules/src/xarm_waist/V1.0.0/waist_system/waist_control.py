#!/usr/bin/env python3
"""ROS2 + terminal position demo for the two-axis waist."""

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


CAN_INTERFACE = "can1"
PITCH_IDS = (0x0A, 0x1A)  # send, receive
YAW_IDS = (0x09, 0x19)
PITCH_LIMIT = (-30.0, 30.0)
YAW_LIMIT = (-90.0, 90.0)
# A powered-off waist can sag beyond its command range. This extra envelope is
# only for taking over and holding the measured pose; it is not a motion limit.
STARTUP_OVERRUN_DEG = 20.0
STARTUP_PITCH_LIMIT = (PITCH_LIMIT[0] - STARTUP_OVERRUN_DEG, PITCH_LIMIT[1] + STARTUP_OVERRUN_DEG)
STARTUP_YAW_LIMIT = (YAW_LIMIT[0] - STARTUP_OVERRUN_DEG, YAW_LIMIT[1] + STARTUP_OVERRUN_DEG)
PITCH_KP, PITCH_KD = 500.0, 5.0
YAW_KP, YAW_KD = 400.0, 5.0
MAX_SPEED = 20.0  # deg/s
CONTROL_HZ = 500.0
RECV_TIMEOUT_US = 200
MAX_CYCLE_ERRORS = 5
LOCK_PATH = "/tmp/k1-can1-waist-09-0a.lock"


def angle_error(pitch, yaw):
    if not math.isfinite(pitch) or not math.isfinite(yaw):
        return "angles must be finite numbers"
    if not PITCH_LIMIT[0] <= pitch <= PITCH_LIMIT[1]:
        return f"pitch must be in {PITCH_LIMIT} deg"
    if not YAW_LIMIT[0] <= yaw <= YAW_LIMIT[1]:
        return f"yaw must be in {YAW_LIMIT} deg"
    return None


def startup_feedback_error(pitch, yaw):
    """Accept modest gravity sag, but reject missing or implausible feedback."""
    if not math.isfinite(pitch) or not math.isfinite(yaw):
        return "initial pitch and yaw must be finite"
    if not STARTUP_PITCH_LIMIT[0] <= pitch <= STARTUP_PITCH_LIMIT[1]:
        return f"initial pitch must be in {STARTUP_PITCH_LIMIT} deg"
    if not STARTUP_YAW_LIMIT[0] <= yaw <= STARTUP_YAW_LIMIT[1]:
        return f"initial yaw must be in {STARTUP_YAW_LIMIT} deg"
    return None


class WaistDriver:
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
                raise RuntimeError("another waist controller owns can1 IDs 0x09/0x0A") from error
            self.xarm = xarm_can.XArm(CAN_INTERFACE, True)
            self.xarm.init_arm_motors(
                [xarm_can.MotorType.DM10422P, xarm_can.MotorType.DM8009],
                [PITCH_IDS[0], YAW_IDS[0]],
                [PITCH_IDS[1], YAW_IDS[1]],
            )
            self.arm = self.xarm.get_arm()
            motor_index = {
                motor.get_send_can_id(): i
                for i, motor in enumerate(self.arm.get_motors())
            }
            self.pitch_index = motor_index[PITCH_IDS[0]]
            self.yaw_index = motor_index[YAW_IDS[0]]
            self.xarm.set_callback_mode_all(xarm_can.CallbackMode.STATE)

            for _ in range(10):
                try:
                    self.initial_position = self.read(2000)
                    break
                except RuntimeError:
                    time.sleep(0.01)
            if self.initial_position is None:
                raise RuntimeError("no valid motor feedback while motors are disabled")

            error = startup_feedback_error(*self.initial_position)
            if error:
                raise RuntimeError(f"unsafe initial motor feedback: {error}")

            self.write(*self.initial_position, gains=False)
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
        pitch = motors[self.pitch_index]
        yaw = motors[self.yaw_index]
        if not all(m.get_state_tmos() or m.get_state_trotor() for m in (pitch, yaw)):
            raise RuntimeError("motor feedback not received")
        return math.degrees(pitch.get_position()), math.degrees(yaw.get_position())

    def write(self, pitch_deg, yaw_deg, gains=True):
        pitch_kp, pitch_kd = (PITCH_KP, PITCH_KD) if gains else (0.0, 0.0)
        yaw_kp, yaw_kd = (YAW_KP, YAW_KD) if gains else (0.0, 0.0)
        self.arm.mit_control_one(
            self.pitch_index,
            xarm_can.MITParam(
                pitch_kp, pitch_kd, math.radians(pitch_deg), 0.0, 0.0
            ),
        )
        self.arm.mit_control_one(
            self.yaw_index,
            xarm_can.MITParam(
                yaw_kp, yaw_kd, math.radians(yaw_deg), 0.0, 0.0
            ),
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


class WaistNode(Node):
    def __init__(self):
        super().__init__("waist_control")
        self.lock = threading.Lock()
        self.target = None
        self.command = None
        self.feedback = None
        self.last_time = time.monotonic()
        self.cycle_errors = 0
        self.closed = False

        self.pub = self.create_publisher(Float64MultiArray, "/waist/state", 10)
        self.sub = self.create_subscription(
            Float64MultiArray, "/waist/cmd", self.ros_command, 10
        )
        self.driver = WaistDriver()
        try:
            self.command = self.driver.initial_position
            self.target = self.command
            self.feedback = self.command
            self.timer = self.create_timer(1.0 / CONTROL_HZ, self.control_cycle)
        except Exception:
            self.driver.close()
            raise

        self.get_logger().info(
            f"Waist ready at pitch={self.command[0]:.2f}, yaw={self.command[1]:.2f} deg; "
            "input [pitch_deg, yaw_deg], Ctrl+C to exit"
        )
        if angle_error(*self.command):
            self.get_logger().warning(
                "Initial waist feedback is outside the command limits; holding the measured "
                "position without automatic recovery. New targets must remain inside the "
                f"normal pitch {PITCH_LIMIT} and yaw {YAW_LIMIT} deg limits."
            )

    def set_target(self, pitch, yaw, source="terminal"):
        error = angle_error(pitch, yaw)
        if error:
            self.get_logger().warning(f"Rejected {source} command: {error}")
            return
        with self.lock:
            self.target = (pitch, yaw)
        self.get_logger().info(f"Target: pitch={pitch:.1f}, yaw={yaw:.1f} deg")

    def ros_command(self, msg):
        if len(msg.data) != 2:
            self.get_logger().warning("Expected /waist/cmd data: [pitch_deg, yaw_deg]")
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
                raise RuntimeError("waist disabled after repeated control errors") from exc

    def close(self):
        if not self.closed:
            self.closed = True
            self.get_logger().info("Disabling waist motors")
            self.driver.close()


def terminal_loop(node):
    print("Input: pitch_deg yaw_deg, or state. Ctrl+C exits.")
    while rclpy.ok():
        try:
            parts = input("waist> ").replace(",", " ").split()
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
        node = WaistNode()
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
