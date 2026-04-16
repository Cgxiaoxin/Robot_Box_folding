"""轨迹引擎模块测试：覆盖存取、删除、合并、时长计算。"""

from pathlib import Path

from backend import trajectory_engine as te


def _use_tmp_trajectory_dir(monkeypatch, tmp_path: Path):
    # 将轨迹目录重定向到 pytest 临时目录，避免污染真实数据
    monkeypatch.setattr(te, "TRAJECTORIES_DIR", tmp_path)


def test_save_and_load_roundtrip(monkeypatch, tmp_path):
    # 核心回归：保存后可完整读回关键字段
    _use_tmp_trajectory_dir(monkeypatch, tmp_path)
    engine = te.TrajectoryEngine()

    trajectory = te.Trajectory(
        name="demo",
        description="roundtrip",
        points=[
            te.TrajectoryPoint(name="p1", positions={"51": 0.1}, delay=1.2),
            te.TrajectoryPoint(name="p2", positions={"51": 0.2}, delay=0.8),
        ],
        loop=True,
        speed_multiplier=1.5,
    )

    assert engine.save(trajectory, "demo.json")
    loaded = engine.load("demo.json")

    assert loaded is not None
    assert loaded.name == "demo"
    assert loaded.loop is True
    assert loaded.speed_multiplier == 1.5
    assert len(loaded.points) == 2
    assert loaded.points[0].positions["51"] == 0.1


def test_list_trajectories_and_delete(monkeypatch, tmp_path):
    # 核心回归：列表元信息正确，删除行为符合预期
    _use_tmp_trajectory_dir(monkeypatch, tmp_path)
    engine = te.TrajectoryEngine()

    traj = te.Trajectory(name="to_delete", points=[te.TrajectoryPoint(name="p", positions={"51": 0.0})])
    assert engine.save(traj, "to_delete.json")

    listed = engine.list_trajectories()
    assert len(listed) == 1
    assert listed[0]["filename"] == "to_delete.json"
    assert listed[0]["points_count"] == 1

    assert engine.delete("to_delete.json") is True
    assert engine.delete("to_delete.json") is False


def test_merge_and_duration(monkeypatch, tmp_path):
    # 核心回归：多轨迹合并与时长计算逻辑
    _use_tmp_trajectory_dir(monkeypatch, tmp_path)
    engine = te.TrajectoryEngine()

    t1 = te.Trajectory(
        name="t1",
        points=[te.TrajectoryPoint(name="a", positions={"51": 0.0}, delay=1.0)],
        speed_multiplier=2.0,
    )
    t2 = te.Trajectory(
        name="t2",
        points=[te.TrajectoryPoint(name="b", positions={"51": 1.0}, delay=2.0)],
    )

    assert engine.save(t1, "t1.json")
    assert engine.save(t2, "t2.json")

    merged = engine.merge_trajectories(["t1.json", "t2.json"], "merged")
    assert merged is not None
    assert merged.name == "merged"
    assert len(merged.points) == 2

    duration = engine.calculate_duration(t1)
    assert duration == 0.5
