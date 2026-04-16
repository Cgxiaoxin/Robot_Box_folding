#!/bin/bash
# ============================================
#  机械臂演出控制系统 - Linux启动脚本
# ============================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "============================================"
echo "  机械臂演出控制系统"
echo "============================================"
echo ""

# ---- 杀掉旧的 app.py 进程，防止多实例 ----
OLD_PIDS=$(lsof -ti :5000 2>/dev/null)
if [ -n "$OLD_PIDS" ]; then
    echo "[!] 检测到端口 5000 已被占用 (PID: $OLD_PIDS)，正在停止旧进程..."
    kill -9 $OLD_PIDS 2>/dev/null
    sleep 1
fi

# ---- 清除 Python 缓存，确保代码修改立即生效 ----
find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null
find . -name "*.pyc" -delete 2>/dev/null

# ---- 确定 Python 运行环境 ----
if command -v uv &> /dev/null; then
    PYTHON_CMD="uv run python"
    echo "[✓] 使用 uv 管理的虚拟环境"
else
    PYTHON_CMD="python3"
    echo "[!] 未找到 uv，使用系统 python3（可能缺少依赖）"
fi

# 检查依赖
echo "[1/3] 检查依赖..."
if command -v uv &> /dev/null; then
    uv sync --quiet 2>/dev/null || echo "[!] uv sync 失败，继续尝试..."
else
    $PYTHON_CMD -c "import flask; import socketio; import eventlet" 2>/dev/null
    if [ $? -ne 0 ]; then
        echo "[警告] 缺少依赖，正在安装..."
        pip install -r requirements.txt
    fi
fi

# 检查CAN接口
echo "[2/3] 检查CAN接口..."

CAN0_OK=false
CAN1_OK=false

if ip link show can0 &> /dev/null; then
    if ip link show can0 | grep -q "state UP"; then
        echo "  ✓ can0 已启动"
        CAN0_OK=true
    else
        echo "  ⚠ can0 未启动"
    fi
else
    echo "  ⚠ 未检测到 can0 接口"
fi

if ip link show can1 &> /dev/null; then
    if ip link show can1 | grep -q "state UP"; then
        echo "  ✓ can1 已启动"
        CAN1_OK=true
    else
        echo "  ⚠ can1 未启动"
    fi
else
    echo "  ⚠ 未检测到 can1 接口"
fi

# 如果CAN接口未启动，提示用户
if [ "$CAN0_OK" = false ] || [ "$CAN1_OK" = false ]; then
    echo ""
    echo "  ⚠️  CAN 接口未完全启动，机械臂将无法通讯！"
    echo ""
    echo "  快速修复："
    echo "    ./setup_can.sh"
    echo ""
    read -p "  是否继续启动服务？(y/N) " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "  已取消启动"
        exit 1
    fi
fi

# 启动服务
echo "[3/3] 启动Web服务..."
echo ""
echo "  访问地址: http://127.0.0.1:5000"
echo "  运行环境: $PYTHON_CMD"
echo "  按 Ctrl+C 停止服务"
echo ""

# 后台启动浏览器 (延迟2秒)
(sleep 2 && xdg-open http://127.0.0.1:5000 2>/dev/null || open http://127.0.0.1:5000 2>/dev/null || true) &

# 启动Flask应用（使用正确的Python环境）
$PYTHON_CMD backend/app.py

