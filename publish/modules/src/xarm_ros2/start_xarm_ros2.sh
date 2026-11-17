#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

# Script is expected to run from publish/modules/src/xarm_ros2.
# Kept in src/xarm_ros2 and copied to publish by build script.
PUBLISH_DIR=$(cd "$SCRIPT_DIR/../../.." && pwd)
WS_DIR="$PUBLISH_DIR/modules"
INSTALL_DIR="$WS_DIR/install"
SDK_ROOT="$PUBLISH_DIR/xarm_can/package"
SDK_LIB_DIR="$SDK_ROOT/lib"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
MODE="single"
SCENE="control"
CONTROLLER="joint_trajectory_controller"
RVIZ="false"
DRY_RUN=false
EXTRA=()
usage() {
  cat <<'EOF'
统一入口：bash start_xarm_ros2.sh [参数...]
  mode:=single|bimanual          单臂/双臂（默认 single）
  scene:=control|moveit          基础控制/MoveIt+RViz（默认 control）
  hardware:=fake|real            假硬件/真机（默认 real）
  controller:=trajectory|position  轨迹/直接位置（默认 trajectory）
  rviz:=true|false               基础控制的显示开关；MoveIt 固定开启
  can_interface:=can0 single_side:=left|right  单臂必填
  right_can_interface:=... left_can_interface:=... 双臂真机必填
  --dry-run                     只打印启动命令，不连接设备
MoveIt 当前只支持 bimanual + trajectory；左右臂分别规划。
EOF
}
fail() { echo "参数错误：$*" >&2; exit 2; }
for arg in "$@"; do
  case "$arg" in
    -h|--help) usage; exit 0 ;;
    --dry-run) DRY_RUN=true ;;
    mode:=*) MODE="${arg#*:=}" ;;
    scene:=*) SCENE="${arg#*:=}" ;;
    hardware:=fake) EXTRA+=("use_fake_hardware:=true") ;;
    hardware:=real) EXTRA+=("use_fake_hardware:=false") ;;
    hardware:=*) fail "$arg" ;;
    controller:=trajectory) CONTROLLER="joint_trajectory_controller" ;;
    controller:=position) CONTROLLER="forward_position_controller" ;;
    controller:=*) fail "$arg" ;;
    use_fake_hardware:=*|robot_controller:=*|start_rviz:=*) fail "已移除旧参数 $arg；请使用 hardware、controller、rviz" ;;
    rviz:=*) RVIZ="${arg#*:=}" ;;
    can_interface:=*|single_side:=*|right_can_interface:=*|left_can_interface:=*|arm_type:=*|description_package:=*|description_file:=*|runtime_config_package:=*|controllers_file:=*) EXTRA+=("$arg") ;;
    *:=*) fail "未知或不支持的参数 $arg；使用 --help 查看帮助" ;;
    *) fail "未知参数 $arg；使用 --help 查看帮助" ;;
  esac
done
[[ "$MODE" == single || "$MODE" == bimanual ]] || fail "mode=$MODE"
[[ "$SCENE" == control || "$SCENE" == moveit ]] || fail "scene=$SCENE"
[[ "$RVIZ" == true || "$RVIZ" == false ]] || fail "rviz=$RVIZ"
[[ "$CONTROLLER" == joint_trajectory_controller || "$CONTROLLER" == forward_position_controller ]] || fail "controller=$CONTROLLER"
FAKE=false
RIGHT_CAN=""
LEFT_CAN=""
SINGLE_CAN=""
SIDE=""
for arg in "${EXTRA[@]}"; do
  case "$arg" in
    use_fake_hardware:=*) FAKE="${arg#*:=}" ;;
    right_can_interface:=*) RIGHT_CAN="${arg#*:=}" ;;
    left_can_interface:=*) LEFT_CAN="${arg#*:=}" ;;
    can_interface:=*) SINGLE_CAN="${arg#*:=}" ;;
    single_side:=*) SIDE="${arg#*:=}" ;;
  esac
done
[[ "$FAKE" == true || "$FAKE" == false ]] || fail "use_fake_hardware=$FAKE"
if [[ "$MODE" == single ]]; then
  [[ -n "$SINGLE_CAN" && ( "$SIDE" == left || "$SIDE" == right ) ]] || fail "单臂请显式指定 can_interface 和 single_side:=left|right"
fi
if [[ "$MODE" == bimanual && "$FAKE" == false ]]; then
  [[ -n "$RIGHT_CAN" && -n "$LEFT_CAN" ]] || fail "双臂真机请显式指定 right_can_interface 和 left_can_interface"
  [[ "$RIGHT_CAN" != "$LEFT_CAN" ]] || fail "左右臂不能使用同一个 CAN 接口"
fi
if [[ "$SCENE" == moveit ]]; then
  [[ "$MODE" == bimanual && "$CONTROLLER" == joint_trajectory_controller ]] || fail "MoveIt 需要 mode:=bimanual controller:=trajectory"
  CMD=(ros2 launch xarm_bimanual_moveit_config demo.launch.py)
else
  if [[ "$MODE" == single ]]; then
    CMD=(ros2 launch xarm_bringup xarm.launch.py "start_rviz:=$RVIZ")
  else
    [[ "$RVIZ" == false ]] || fail "双臂可视化请使用 scene:=moveit"
    CMD=(ros2 launch xarm_bringup xarm.bimanual.launch.py)
  fi
fi
CMD+=("arm_type:=v10" "robot_controller:=$CONTROLLER" "${EXTRA[@]}")
if "$DRY_RUN"; then
  printf '%q ' "${CMD[@]}"
  printf '\n'
  exit 0
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup not found: $ROS_SETUP"
  exit 1
fi

if [[ ! -f "$INSTALL_DIR/setup.bash" ]]; then
  echo "publish install not found: $INSTALL_DIR/setup.bash"
  echo "Please build in publish/modules first:"
  echo "  cd $WS_DIR"
  echo "  source /opt/ros/$ROS_DISTRO_NAME/setup.bash"
  echo "  export XARM_SDK_ROOT=$PUBLISH_DIR/xarm_can/package"
  echo "  colcon build --base-paths src/xarm_ros2 src/xarm_description src/xarm_teleop --packages-select xarm xarm_bringup xarm_hardware xarm_description xarm_bimanual_moveit_config xarm_teleop --cmake-args -DXARM_SDK_ROOT=$PUBLISH_DIR/xarm_can/package"
  exit 1
fi

ARCH=$(uname -m)
if [[ -f "$SDK_LIB_DIR/$ARCH/libxarm_can_sdk.so" ]]; then
  SDK_LIB_DIR="$SDK_LIB_DIR/$ARCH"
fi
if [[ ! -f "$SDK_LIB_DIR/libxarm_can_sdk.so" ]]; then
  echo "SDK so not found: $SDK_LIB_DIR/libxarm_can_sdk.so"
  exit 1
fi
export LD_LIBRARY_PATH="$SDK_LIB_DIR:${LD_LIBRARY_PATH:-}"

set +u
source "$ROS_SETUP"
source "$INSTALL_DIR/setup.bash"
set -u

for pkg_prefix in \
  "$INSTALL_DIR/xarm_description" \
  "$INSTALL_DIR/xarm_hardware" \
  "$INSTALL_DIR/xarm_bringup"
do
  if [[ -d "$pkg_prefix" ]]; then
    export AMENT_PREFIX_PATH="$pkg_prefix:${AMENT_PREFIX_PATH:-}"
    export CMAKE_PREFIX_PATH="$pkg_prefix:${CMAKE_PREFIX_PATH:-}"
  fi
done

exec "${CMD[@]}"
