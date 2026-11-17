#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SDK_ROOT=$(cd "$SCRIPT_DIR/../../../../xarm_can/package" && pwd)
ARCH=$(uname -m)
OUTPUT=${1:-"$SCRIPT_DIR/ramp_action_player"}
CXX_BIN=${CXX:-g++}

case "$ARCH" in
    x86_64|aarch64) ;;
    *)
        echo "[ERROR] Unsupported architecture: $ARCH" >&2
        exit 1
        ;;
esac

SDK_LIB_DIR="$SDK_ROOT/lib/$ARCH"
SDK_LIB="$SDK_LIB_DIR/libxarm_can_sdk.so"

if ! command -v "$CXX_BIN" >/dev/null 2>&1; then
    echo "[ERROR] C++ compiler not found: $CXX_BIN" >&2
    exit 1
fi
if [ ! -f "$SDK_ROOT/include/xarm_can_sdk.h" ]; then
    echo "[ERROR] SDK header not found: $SDK_ROOT/include/xarm_can_sdk.h" >&2
    exit 1
fi
if [ ! -f "$SDK_LIB" ]; then
    echo "[ERROR] SDK library not found: $SDK_LIB" >&2
    exit 1
fi

"$CXX_BIN" \
    -std=c++17 -O2 -Wall -Wextra -Wpedantic \
    -I"$SDK_ROOT/include" \
    "$SCRIPT_DIR/ramp_action_player.cpp" \
    -L"$SDK_LIB_DIR" -Wl,-rpath,"$SDK_LIB_DIR" \
    -lxarm_can_sdk -pthread \
    -o "$OUTPUT"

echo "[PASS] Built $OUTPUT ($ARCH)"
