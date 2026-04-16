from linkerbot import L6

with L6(side="left", interface_name="can2") as hand:
    # 设置角度
    hand.angle.set_angles([90.0, 90.0, 90.0, 90.0, 90.0, 90.0])

    # 读取当前角度
    data = hand.angle.get_blocking(timeout_ms=500)
    print(f"当前角度：{data.angles.to_list()}")

    # 渐进移动
    for i in range(0, 101, 10):
        hand.angle.set_angles([float(i)] * 6)