#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PUBLISH_DIR=$(cd "$SCRIPT_DIR/../../../../.." && pwd)
SDK_ARCH="$(uname -m)"
SDK_RUNTIME="${K1_HEAD_WAIST_RUNTIME_DIR:-$PUBLISH_DIR/xarm_can/head_waist/python/prebuilt/linux-$SDK_ARCH}"
PYTHON_BIN="${PYTHON_BIN:-/usr/bin/python3}"
if [[ -n "${ROS_SETUP:-}" ]]; then
    :
elif [[ -n "${ROS_DISTRO:-}" ]]; then
    ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    ROS_SETUP=/opt/ros/jazzy/setup.bash
else
    ROS_SETUP=/opt/ros/humble/setup.bash
fi
command -v "$PYTHON_BIN" >/dev/null || {
    echo "[ERROR] Python interpreter not found: $PYTHON_BIN" >&2
    exit 1
}
[[ -f "$ROS_SETUP" ]] || {
    echo "[ERROR] ROS setup not found: $ROS_SETUP" >&2
    exit 1
}

source "$ROS_SETUP"
export PYTHONPATH="$SDK_RUNTIME${PYTHONPATH:+:$PYTHONPATH}"

ip link show can0 | grep -q '<[^>]*UP' || {
    echo "[ERROR] can0 is missing or not UP" >&2
    exit 1
}

"$PYTHON_BIN" -c 'import pathlib, sys, rclpy, k1_head_waist_can as xarm_can; xarm_can.MotorType.DM10422P; xarm_can.XArm; assert pathlib.Path(xarm_can.__file__).resolve().parent == (pathlib.Path(sys.argv[1]) / "k1_head_waist_can").resolve()' "$SDK_RUNTIME" 2>/dev/null || {
    echo "[ERROR] $PYTHON_BIN needs rclpy and the bundled K1 head/waist SDK; use Linux x86_64/aarch64 with Python 3.10 and follow docs/用户使用文档/06_K1头腰使用文档.md" >&2
    exit 1
}

exec "$PYTHON_BIN" "$SCRIPT_DIR/head_control.py"
