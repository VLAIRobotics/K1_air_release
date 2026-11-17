#!/bin/bash
# XArm Web UI 启动脚本

# 设置颜色输出
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}🤖 XArm Web UI 启动脚本${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# 检查Python环境
echo -e "${BLUE}📦 检查Python环境...${NC}"
if ! command -v python3 &> /dev/null; then
    echo -e "${RED}❌ 未找到Python3，请先安装Python3${NC}"
    exit 1
fi

echo -e "✓ Python版本: $(python3 --version)"
echo ""

# 检查并安装依赖
echo -e "${BLUE}📦 检查依赖包...${NC}"
if [ -f "requirements.txt" ]; then
    echo "正在检查/安装依赖..."
    python3 -m pip install -q -r requirements.txt
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ 依赖包已就绪${NC}"
    else
        echo -e "${RED}❌ 依赖安装失败${NC}"
        exit 1
    fi
else
    echo -e "${RED}❌ 未找到 requirements.txt${NC}"
    exit 1
fi
echo ""

# 检查ROS2环境
echo -e "${BLUE}🤖 检查ROS2环境...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}⚠️  警告: 未检测到ROS2环境变量${NC}"
    echo "请先执行: source /opt/ros/humble/setup.bash"
    echo "以及从 SDK 根目录执行: source ./publish/modules/install/setup.bash"
    echo ""
    echo "是否继续启动 Web UI? (y/n)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        exit 1
    fi
else
    echo -e "✓ ROS发行版: ${GREEN}$ROS_DISTRO${NC}"
fi
echo ""

# 与后端使用相同的监听配置；不依赖 lsof，也不终止其他程序。
export XARM_UI_HOST="${XARM_UI_HOST:-0.0.0.0}"
export XARM_UI_PORT="${XARM_UI_PORT:-5000}"
echo -e "${BLUE}🔍 检查端口${XARM_UI_PORT}...${NC}"
if ! python3 - <<'PYPORT'
import os
import socket
import sys
host = os.environ["XARM_UI_HOST"]
try:
    port = int(os.environ["XARM_UI_PORT"])
    if not 1 <= port <= 65535:
        raise ValueError("端口必须在 1–65535 之间")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
except (OSError, ValueError) as exc:
    print(f"无法监听 {host}:{os.environ['XARM_UI_PORT']}: {exc}", file=sys.stderr)
    sys.exit(1)
PYPORT
then
    echo -e "${RED}❌ 监听地址不可用${NC}"
    echo "查看占用进程: ss -ltnp"
    echo "换用其他端口: XARM_UI_PORT=5001 bash start_ui.sh"
    exit 1
fi
echo -e "${GREEN}✓ 端口${XARM_UI_PORT}可用${NC}"
echo ""

# 显示项目信息
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}📁 项目信息${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo "工作目录: $SCRIPT_DIR"
echo "访问地址: http://localhost:${XARM_UI_PORT}"
echo "日志级别: INFO"
echo ""

# 启动服务器
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}🚀 启动 Web UI 服务器...${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""
echo -e "${GREEN}提示: 按 Ctrl+C 停止服务器${NC}"
echo ""

# 启动Python应用
python3 app.py
server_status=$?

# 清理
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}👋 服务器已停止${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"

exit "$server_status"
