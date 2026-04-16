@echo off
chcp 65001 >nul
REM ============================================
REM  机械臂演出控制系统 - Windows启动脚本
REM ============================================

cd /d %~dp0

echo ============================================
echo   机械臂演出控制系统
echo ============================================
echo.

REM 检查Python
python --version >nul 2>&1
if errorlevel 1 (
    echo [错误] 未找到 Python
    pause
    exit /b 1
)

REM 检查依赖
echo [1/2] 检查依赖...
python -c "import flask; import socketio; import eventlet" 2>nul
if errorlevel 1 (
    echo [警告] 缺少依赖，正在安装...
    pip install -r requirements.txt
)

REM 启动服务
echo [2/2] 启动Web服务...
echo.
echo   访问地址: http://127.0.0.1:5000
echo   按 Ctrl+C 停止服务
echo.

REM 启动浏览器
start "" http://127.0.0.1:5000

REM 启动Flask应用
python backend\app.py

pause
