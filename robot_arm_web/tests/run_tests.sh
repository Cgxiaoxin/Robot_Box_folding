#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

# 关闭外部 pytest 自动插件，避免本机环境（如 ROS）干扰测试收集
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q -c "${SCRIPT_DIR}/pytest.ini" "${PROJECT_ROOT}/tests" "$@"

