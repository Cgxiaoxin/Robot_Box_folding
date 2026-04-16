#!/bin/bash
# CAN 接口完全重置脚本（多通道版本）

set -u

BITRATE="${1:-1000000}"
TXQLEN="${2:-100}"

echo "正在重置 CAN 接口... (bitrate=${BITRATE}, txqueuelen=${TXQLEN})"
echo ""

mapfile -t CAN_IFACES < <(ip -br link show type can 2>/dev/null | awk '{print $1}')

if [ "${#CAN_IFACES[@]}" -eq 0 ]; then
    echo "未发现任何 CAN 接口，请先检查 USB-CAN 盒和驱动。"
    exit 1
fi

echo "发现 CAN 接口: ${CAN_IFACES[*]}"
echo ""

echo "[1/5] 停止占用 CAN 的进程..."
sudo pkill -9 candump 2>/dev/null || true
sudo pkill -9 cansniffer 2>/dev/null || true
sudo pkill -9 cansend 2>/dev/null || true
echo "  ✓ 已清理进程"
echo ""

echo "[2/5] 关闭所有 CAN 接口..."
for iface in "${CAN_IFACES[@]}"; do
    sudo ip link set "$iface" down 2>/dev/null || true
done
sleep 1
echo "  ✓ 已关闭: ${CAN_IFACES[*]}"
echo ""

echo "[3/5] 重新配置并拉起所有 CAN 接口..."
for iface in "${CAN_IFACES[@]}"; do
    sudo ip link set "$iface" down 2>/dev/null || true
    if sudo ip link set "$iface" type can bitrate "$BITRATE" 2>/dev/null \
        && sudo ip link set "$iface" txqueuelen "$TXQLEN" 2>/dev/null \
        && sudo ip link set "$iface" up 2>/dev/null; then
        echo "  ✓ $iface 已拉起"
    else
        echo "  ✗ $iface 拉起失败"
    fi
done
sleep 1
echo ""

echo "[4/5] 验证链路状态..."
for iface in "${CAN_IFACES[@]}"; do
    state=$(ip -details link show "$iface" 2>/dev/null | grep -oP 'state \K\w+' | head -n1 || echo "ERROR")
    rx_packets=$(ip -s link show "$iface" 2>/dev/null | awk '/RX:/ {getline; print $2}' | head -n1)
    tx_packets=$(ip -s link show "$iface" 2>/dev/null | awk '/TX:/ {getline; print $2}' | head -n1)
    if [ "$state" = "UP" ]; then
        echo "  ✓ $iface: state=$state RX=${rx_packets:-0} TX=${tx_packets:-0}"
    else
        echo "  ✗ $iface: state=$state RX=${rx_packets:-0} TX=${tx_packets:-0}"
    fi
done
echo ""

echo "[5/5] 建议的下一步"
echo "  1) 扫描灵巧手口位: bash ../src/linkerhand-ros2-sdk/find_linker_hand.sh"
echo "  2) Web 侧扫描双臂口位后，再固定 left_channel/right_channel"
echo ""
echo "CAN 接口重置完成。"
