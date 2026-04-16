"""轨迹插值与同步规划测试：覆盖插值结果与同步延迟逻辑。"""

from backend.trajectory_interpolator import (
    InterpolationType,
    SynchronizedMotionPlanner,
    TrajectoryInterpolator,
    TrajectoryPoint,
    create_interpolator,
)


def test_linear_interpolation_basic_shape():
    # 线性插值应在起终点间单调推进，并包含最终点
    interpolator = TrajectoryInterpolator(interpolation_type=InterpolationType.LINEAR, num_points=4)
    p1 = TrajectoryPoint(positions={51: 0.0}, time=0.0, name="start")
    p2 = TrajectoryPoint(positions={51: 1.0}, time=1.0, name="end")

    points = interpolator.interpolate([p1, p2])

    # 4个插值点 + 原始终点
    assert len(points) == 5
    assert points[0].positions[51] == 0.0
    assert points[-1].positions[51] == 1.0
    assert points[1].positions[51] > points[0].positions[51]


def test_s_curve_interpolation_reaches_target():
    # S 曲线插值至少要保证首末点精确命中
    interpolator = TrajectoryInterpolator(interpolation_type=InterpolationType.S_CURVE, num_points=6)
    p1 = TrajectoryPoint(positions={51: -0.5}, time=0.0, name="a")
    p2 = TrajectoryPoint(positions={51: 0.5}, time=1.0, name="b")

    points = interpolator.interpolate([p1, p2])

    assert len(points) == 7
    assert points[0].positions[51] == -0.5
    assert points[-1].positions[51] == 0.5


def test_sync_planner_delay_calculation():
    # 同步延迟：超出容差部分才需要补偿
    planner = SynchronizedMotionPlanner(sync_tolerance_ms=50.0)
    delays = planner.calculate_sync_delay({"left": 1.00, "right": 1.12})

    # delta=120ms, tolerance=50ms, remaining delay=70ms
    assert delays["left"] == 0
    assert round(delays["right"], 3) == 70.0


def test_create_interpolator_default_fallback():
    # 非法类型时应回退默认 S 曲线
    interpolator = create_interpolator("unknown_type", num_points=10)
    assert interpolator.interpolation_type == InterpolationType.S_CURVE
    assert interpolator.num_points == 10
