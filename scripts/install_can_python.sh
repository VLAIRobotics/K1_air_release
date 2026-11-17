#!/usr/bin/env bash
set -euo pipefail
sdk_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
python_bin=${PYTHON_BIN:-python3}
# A regular pip install replaces metadata from older editable installations.
"$python_bin" -m pip install --force-reinstall --no-deps "$sdk_root/publish/xarm_can/python"
"$python_bin" -c 'import xarm_can; print("CAN Python SDK:", xarm_can.__file__); assert hasattr(xarm_can, "XArm")'

# K1 head/waist is a separate namespace; do not overwrite arm bindings.
"$python_bin" -m pip install --force-reinstall --no-deps "$sdk_root/publish/xarm_can/head_waist/python"
"$python_bin" -c 'import xarm_can, k1_head_waist_can; assert xarm_can.XArm; assert k1_head_waist_can.XArm; assert k1_head_waist_can.MotorType.DM10422P; print("K1 head/waist SDK:", k1_head_waist_can.__file__)'
