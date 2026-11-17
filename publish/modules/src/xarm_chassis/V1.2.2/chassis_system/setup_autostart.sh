#!/bin/bash
#
# 配置开机自启动脚本
# 功能：配置 CAN 接口和 CAN 监听触发器的开机自启动
#

set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

CAN_SETUP_SCRIPT="$SCRIPT_DIR/../../../../../xarm_can/package/libexec/setup_can_interfaces.sh"
CAN_LISTENER_SCRIPT="$SCRIPT_DIR/can_listener_trigger.sh"

CAN_SETUP_SERVICE="/etc/systemd/system/xarm-can-setup.service"
CAN_LISTENER_SERVICE="/etc/systemd/system/xarm-can-listener.service"

echo "========================================"
echo "  配置开机自启动"
echo "========================================"

if [ "$(id -u)" -ne 0 ]; then
    echo "[ERROR] 请以 root 权限运行此脚本 (sudo)"
    exit 1
fi

echo "[INFO] 检查脚本文件..."

if [ ! -f "$CAN_SETUP_SCRIPT" ]; then
    echo "[ERROR] CAN 设置脚本不存在: $CAN_SETUP_SCRIPT"
    exit 1
fi

if [ ! -f "$CAN_LISTENER_SCRIPT" ]; then
    echo "[ERROR] CAN 监听脚本不存在: $CAN_LISTENER_SCRIPT"
    exit 1
fi

echo "[INFO] 确保脚本可执行..."
chmod +x "$CAN_SETUP_SCRIPT"
chmod +x "$CAN_LISTENER_SCRIPT"

echo "[INFO] 创建 CAN 设置服务..."
cat > "$CAN_SETUP_SERVICE" << EOF
[Unit]
Description=XArm CAN Interface Setup
After=network.target
Wants=network.target

[Service]
Type=oneshot
User=vlai
ExecStart=$CAN_SETUP_SCRIPT
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
EOF

echo "[INFO] 创建 CAN 监听服务..."
cat > "$CAN_LISTENER_SERVICE" << EOF
[Unit]
Description=XArm CAN Listener Trigger
After=xarm-can-setup.service
Requires=xarm-can-setup.service

[Service]
Type=simple
User=vlai
WorkingDirectory=$SCRIPT_DIR
ExecStart=$CAN_LISTENER_SCRIPT
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "[INFO] 重新加载 systemd 配置..."
systemctl daemon-reload

echo "[INFO] 启用开机自启动..."
systemctl enable xarm-can-setup.service
systemctl enable xarm-can-listener.service

echo "[INFO] 启动服务..."
systemctl start xarm-can-setup.service
systemctl start xarm-can-listener.service

echo ""
echo "========================================"
echo "  配置完成"
echo "========================================"
echo ""
echo "服务状态:"
echo "----------------------------------------"
systemctl is-enabled xarm-can-setup.service
systemctl is-enabled xarm-can-listener.service
echo ""
echo "运行状态:"
echo "----------------------------------------"
systemctl status xarm-can-setup.service --no-pager | grep -E "Active|Main PID"
echo ""
systemctl status xarm-can-listener.service --no-pager | grep -E "Active|Main PID"
echo ""
echo "========================================"
echo "  如需禁用自启动，请运行:"
echo "    sudo systemctl disable xarm-can-setup.service"
echo "    sudo systemctl disable xarm-can-listener.service"
echo "========================================"