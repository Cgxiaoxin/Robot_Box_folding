# robot_arm_web Test Plan

## Goals
- Protect core business logic from regressions without requiring hardware.
- Keep test runtime fast for daily development.
- Separate hardware-dependent checks into a lightweight smoke layer.

## Test Layers
1. Unit tests (no hardware, default in CI/local)
- `tests/test_trajectory_engine.py`
  - trajectory save/load/delete/list
  - merge and duration calculation
- `tests/test_trajectory_interpolator.py`
  - linear and s-curve interpolation behavior
  - synchronized delay calculation
  - interpolator factory fallback
- `tests/test_safety_controller.py`
  - limit violation detection
  - limits enable/disable behavior
  - safety event log output
- `tests/test_group_controller_logic.py`
  - soft-limit rejection in manual set
  - target-arm routing logic
  - recorded point structure
  - unknown speed preset error path

2. Integration tests (next step)
- Socket.IO event flow in `backend/app.py`
  - connect/init/target switch/set_speed/trajectory lifecycle
  - error branches: missing params, missing trajectory, IK failure
- state broadcast payload schema checks

3. Hardware smoke tests (manual gate)
- connect -> init -> read positions -> jog one joint -> go_to_zero -> emergency_stop
- run before release on real CAN setup

## How To Run
```bash
cd /data/coding_pro/robot_arm_web
python3 -m pip install -r tests/requirements-dev.txt
./tests/run_tests.sh
```

## Notes
- `tests/pytest.ini` restricts discovery to `/tests`.
- Existing SDK tests under `lansi_arm_sdk/python/tests` are intentionally excluded from default run because they currently do not match this repo's active SDK implementation.
