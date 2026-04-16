#!/bin/bash
# CAN 接口配置脚本

echo "正在配置 CAN 接口..."

# 关闭旧的连接
sudo ip link set can0 down 2>/dev/null
sudo ip link set can1 down 2>/dev/null

# 配置并启动 can0（左臂）
echo "启动 can0（左臂）..."
sudo ip link set can0 up type can bitrate 1000000
if [ $? -eq 0 ]; then
    echo "✓ can0 启动成功"
else
    echo "✗ can0 启动失败"
fi

# 配置并启动 can1（右臂）
echo "启动 can1（右臂）..."
sudo ip link set can1 up type can bitrate 1000000
if [ $? -eq 0 ]; then
    echo "✓ can1 启动成功"
else
    echo "✗ can1 启动失败"
fi

# 显示状态
echo ""
echo "CAN 接口状态："
ip link show can0 | grep -E "can0|state"
ip link show can1 | grep -E "can1|state"

echo ""
echo "配置完成！现在可以启动 Web 服务了。"
