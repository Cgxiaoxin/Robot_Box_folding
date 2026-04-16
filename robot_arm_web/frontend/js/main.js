/**
 * 机械臂示教控制系统 - 前端逻辑
 */

// ==================== Socket.IO 连接 ====================

const socket = io({
    reconnectionAttempts: 5,
    reconnectionDelay: 1000,
    reconnectionDelayMax: 10000,
    timeout: 60000,
});

// 电机ID配置
const LEFT_MOTOR_IDS = [51, 52, 53, 54, 55, 56, 57];
const RIGHT_MOTOR_IDS = [61, 62, 63, 64, 65, 66, 67];
const RIGHT_INVERT_MOTORS = new Set([61, 66]);

// 全局状态
let appState = {
    connected: false,
    arms: {
        left: { connected: false, initialized: false, motors: {} },
        right: { connected: false, initialized: false, motors: {} }
    },
    // 由后端附带的软限位信息：{ left: { "51": {min, max}, ...}, right: {...} }
    joint_limits: {
        left: {},
        right: {}
    },
    target: 'both',
    playback: {
        state: 'idle',
        progress: 0,
        trajectory_name: ''
    },
    trajectories: [],
    drag_teach: {
        enabled: false,
        active_session: null,
        arms: {}
    },
    cartesian_live: null
};

const MOTION_ERROR_TEXT = {
    ARM_NOT_CONNECTED: '机械臂未连接，请先连接并初始化',
    IK_UNREACHABLE: '目标位姿不可达，请减小位移或调整姿态',
    IK_LOW_ACCURACY: 'IK 精度不足，请降低速度并接近可操作区',
    SINGULARITY_NEAR: '接近奇异位形，请调整姿态后再试',
    JOINT_LIMIT_REJECTED: '关节超限，命令已拒绝',
    CONSTRAINT_VIOLATION: '速度/加速度参数超范围',
    COMMAND_PREEMPTED: '命令被新命令抢占',
    CAN_TIMEOUT: '通讯超时，请检查总线与供电',
    INTERNAL_ERROR: '内部错误，请查看系统日志',
    COMMAND_IN_PROGRESS: '执行中禁止重复提交，请先取消/暂停或等待完成'
};

let motionState = {
    mode: 'CARTESIAN_PTP',
    activeCommandId: null,
    activeArmId: null,
    lifecycle: 'idle',
    lastError: null,
    jogTimer: null
};

let armConnectInFlight = false;

const MOTION_MODE_META = {
    CARTESIAN_PTP: {
        desc: 'PTP：末端不保证直线，适合快速到位。',
        execText: '执行目标位姿'
    },
    CARTESIAN_LINEAR: {
        desc: 'LINEAR：末端按直线插补运动，路径更可控。',
        execText: '执行直线运动'
    },
    CARTESIAN_JOG: {
        desc: 'JOG：使用下方点动按钮做增量运动（支持长按连续）。',
        execText: '请使用下方点动按钮'
    }
};

// 跟踪正在拖动的滑杆
const draggingSliders = new Set(); // 存储 "arm_id:motor_id"

// 跟踪待发送的位置命令（防抖）
const pendingPositionCommands = new Map(); // key: "arm_id:motor_id", value: {position, timeoutId}

// ==================== Socket.IO 事件处理 ====================

socket.on('connect', () => {
    console.log('[Socket.IO] 已连接到服务器');
    updateConnectionStatus(true);
    addLog('已连接到服务器', 'success');
    restoreChannelInputs();
    loadTrajectories();
    // 连接成功后同步零点显示与速度滑杆
    loadZeroOffset();
    initSpeedSlider();
    socket.emit('state_request', {});
});

socket.on('disconnect', () => {
    console.log('[Socket.IO] 与服务器断开连接');
    updateConnectionStatus(false);
    addLog('与服务器断开连接', 'error');
});

socket.on('reconnect_failed', () => {
    console.error('[Socket.IO] 重连失败，请刷新页面');
    addLog('重连失败，请刷新页面重试', 'error');
    showNotification('与服务器的连接已断开，请刷新页面重新连接', 'error');
});

socket.on('state:update', (state) => {
    appState = { ...appState, ...state };
    updateUI();
    if (window.Viewer3D && typeof window.Viewer3D.updateState === 'function') {
        window.Viewer3D.updateState(appState);
    }
});

socket.on('cartesian:live', (payload) => {
    appState.cartesian_live = payload || null;
    updateCartesianLiveUI();
});

// 接收单电机位置更新（优化后的快速更新）
socket.on('motor:position_update', (data) => {
    const { arm_id, motor_id, position, relative_position } = data;

    // 更新状态
    if (appState.arms?.[arm_id]?.motors?.[motor_id]) {
        appState.arms[arm_id].motors[motor_id].position = position;
        appState.arms[arm_id].motors[motor_id].relative_position = relative_position;
    }

    // 更新UI（仅该电机，跳过正在拖动的）
    const sliderKey = `${arm_id}:${motor_id}`;
    if (!draggingSliders.has(sliderKey)) {
        const slider = document.querySelector(`.joint-slider-small[data-arm="${arm_id}"][data-motor="${motor_id}"]`);
        if (slider) {
            const min = parseFloat(slider.min);
            const max = parseFloat(slider.max);
            const clamped = clamp(relative_position, isFinite(min) ? min : -3.14, isFinite(max) ? max : 3.14);
            slider.value = String(clamped);

            const labelId = arm_id === 'left' ? `pos-left-${motor_id}` : `pos-right-${motor_id}`;
            const labelEl = document.getElementById(labelId);
            if (labelEl) {
                labelEl.textContent = clamped.toFixed(4);
            }
        }
    }
});

socket.on('arm:connected', (data) => {
    armConnectInFlight = false;
    if (data.success) {
        addLog('机械臂连接成功', 'success');
    } else {
        addLog('机械臂连接失败', 'error');
    }
});

socket.on('arm:disconnected', (data) => {
    armConnectInFlight = false;
    addLog(`${data.arm_id} 臂已断开`, 'info');
});

socket.on('arm:initialized', (data) => {
    const results = Object.entries(data.results)
        .map(([k, v]) => `${k}: ${v ? '成功' : '失败'}`)
        .join(', ');
    addLog(`初始化完成 - ${results}`, 'success');
});

socket.on('arm:deactivated', (data) => {
    addLog(`${data.arm_id} 已去使能`, 'info');
});

socket.on('arm:emergency_stopped', () => {
    addLog('!!! 紧急停止已执行 !!!', 'warning');
});

socket.on('trajectory:started', (data) => {
    addLog(`开始播放轨迹: ${data.filename}`, 'info');
});

socket.on('trajectory:progress', (data) => {
    updatePlaybackProgress(data);
});

socket.on('trajectory:paused', () => {
    addLog('轨迹已暂停', 'info');
});

socket.on('trajectory:resumed', () => {
    addLog('轨迹已恢复', 'info');
});

socket.on('trajectory:stopped', () => {
    addLog('轨迹已停止', 'info');
    updatePlaybackProgress({ progress: 0, current: 0, total: 0 });
});

socket.on('drag_teach:enabled', (data) => {
    if (data?.state) {
        appState.drag_teach = data.state;
        updateDragTeachUI();
    }
    addLog('拖拽示教已启用', 'success');
});

socket.on('drag_teach:disabled', (data) => {
    if (data?.state) {
        appState.drag_teach = data.state;
        updateDragTeachUI();
    }
    addLog('拖拽示教已退出', 'info');
});

socket.on('drag_teach:state', (data) => {
    appState.drag_teach = data || appState.drag_teach;
    updateDragTeachUI();
});

socket.on('drag_teach:keyframe_recorded', (data) => {
    const delay = typeof data?.delay === 'number' ? `${data.delay.toFixed(2)}s` : '-';
    addLog(`Drag Teach 记录关键帧: ${data?.name || '(unnamed)'} / delay=${delay}`, 'success');
});

socket.on('drag_teach:segment_started', (data) => {
    appState.drag_teach = {
        ...(appState.drag_teach || {}),
        active_session: data
    };
    updateDragTeachUI();
    addLog(`Drag Teach 段开始录制: ${data?.name || ''}`, 'info');
});

socket.on('drag_teach:segment_stopped', (data) => {
    addLog(`Drag Teach 段停止: ${data?.name || ''}`, 'info');
});

socket.on('drag_teach:segment_saved', (data) => {
    addLog(`Drag Teach 段已保存: ${data?.filename || ''}`, 'success');
    loadTrajectories();
});

socket.on('drag_teach:segments', (data) => {
    const segments = data?.segments || [];
    addLog(`Drag Teach 段列表已刷新，共 ${segments.length} 个`, 'info');
});

socket.on('error', (data) => {
    if (String(data.message || '').includes('连接')) {
        armConnectInFlight = false;
    }
    addLog(`错误: ${data.message}`, 'error');
    showNotification(data.message, 'error');
});

socket.on('motion:accepted', (data) => {
    motionState.activeCommandId = data.command_id;
    motionState.activeArmId = data.arm_id || motionState.activeArmId;
    motionState.lifecycle = 'accepted';
    updateMotionUIState();
    addLog(`命令已受理: ${data.command_id}`, 'info');
});

socket.on('motion:status', (data) => {
    if (data.command_id && motionState.activeCommandId && data.command_id !== motionState.activeCommandId) return;
    if (data.state) motionState.lifecycle = data.state;
    const progress = Number(data.progress || 0);
    const fill = document.getElementById('motion-progress-fill');
    const text = document.getElementById('motion-progress-text');
    if (fill) fill.style.width = `${Math.max(0, Math.min(100, progress * 100))}%`;
    if (text) text.textContent = `${data.state || 'running'} ${Math.round(progress * 100)}%`;
    if (data.current_pose) {
        renderMotionCurrentPose(data.current_pose);
    }
    if (typeof data.pose_error_mm === 'number') {
        const errEl = document.getElementById('motion-error-text');
        if (errEl) errEl.textContent = `${data.pose_error_mm.toFixed(2)} mm`;
    }
    updateMotionUIState();
});

socket.on('motion:paused', () => {
    motionState.lifecycle = 'paused';
    updateMotionUIState();
});

socket.on('motion:resumed', () => {
    motionState.lifecycle = 'running';
    updateMotionUIState();
});

socket.on('motion:completed', (data) => {
    if (data.command_id && motionState.activeCommandId && data.command_id !== motionState.activeCommandId) return;
    motionState.lifecycle = 'completed';
    motionState.activeCommandId = null;
    motionState.activeArmId = null;
    const text = document.getElementById('motion-progress-text');
    if (text) text.textContent = `completed (${data.duration_ms || 0}ms)`;
    addLog(`命令完成: ${data.command_id}`, 'success');
    updateMotionUIState();
});

socket.on('motion:failed', (data) => {
    if (data.command_id && motionState.activeCommandId && data.command_id !== motionState.activeCommandId) return;
    motionState.lifecycle = 'failed';
    motionState.activeCommandId = null;
    motionState.activeArmId = null;
    motionState.lastError = data.error_code || 'INTERNAL_ERROR';
    const msg = `[${motionState.lastError}] ${MOTION_ERROR_TEXT[motionState.lastError] || data.message || '执行失败'}`;
    const errEl = document.getElementById('motion-last-error');
    if (errEl) errEl.textContent = msg;
    addLog(msg, 'error');
    showNotification(msg, 'error');
    updateMotionUIState();
});

socket.on('motion:rejected', (data) => {
    motionState.lastError = data.error_code || 'CONSTRAINT_VIOLATION';
    const msg = `[${motionState.lastError}] ${MOTION_ERROR_TEXT[motionState.lastError] || data.message || '命令被拒绝'}`;
    const errEl = document.getElementById('motion-last-error');
    if (errEl) errEl.textContent = msg;
    addLog(msg, 'warning');
    showNotification(msg, 'warning');
    updateMotionUIState();
});

socket.on('motion:cancelled', (data) => {
    if (data.command_id && motionState.activeCommandId && data.command_id !== motionState.activeCommandId) return;
    motionState.lifecycle = 'cancelled';
    motionState.activeCommandId = null;
    motionState.activeArmId = null;
    addLog(`命令已取消: ${data.command_id}`, 'info');
    updateMotionUIState();
});

socket.on('motion:safe_abort', (data) => {
    const action = data?.action || 'unknown';
    const reason = data?.reason || 'unknown';
    const msg = `触发安全回退: action=${action}, reason=${reason}`;
    const errEl = document.getElementById('motion-last-error');
    if (errEl) errEl.textContent = msg;
    addLog(msg, 'warning');
    showNotification(msg, 'warning');
});

socket.on('arm:disabled', (data) => {
    addLog(`电机已关闭: ${data?.arm_id || 'target'}`, 'info');
});

socket.on('arm:went_to_zero', (data) => {
    addLog(`已回零: ${data?.arm_id || 'target'}`, 'info');
});

socket.on('manual:position_set', (data) => {
    if (data?.success) return;
    addLog('手动位置下发失败', 'warning');
});

socket.on('teaching:recorded', (data) => {
    const name = data?.point?.name || 'unnamed';
    addLog(`记录当前位置成功: ${name}`, 'success');
});

socket.on('freedrive:enabled', (data) => {
    addLog(`自由拖动已启用: ${data?.arm_id || 'target'}`, 'info');
});

socket.on('freedrive:disabled', (data) => {
    addLog(`自由拖动已关闭: ${data?.arm_id || 'target'}`, 'info');
});

socket.on('execution:result', (data) => {
    const ok = !!data?.success;
    addLog(`execution:result ${ok ? '成功' : '失败'} action=${data?.action_id || '-'}`, ok ? 'success' : 'error');
});

socket.on('zero:calibrated', (data) => {
    const result = data?.result || {};
    const parts = [];
    let hasError = false;
    for (const [arm, status] of Object.entries(result)) {
        if (typeof status === 'string' && status.startsWith('error')) hasError = true;
        parts.push(`${arm}: ${status}`);
    }
    const msg = parts.length ? `零点标定结果：${parts.join(', ')}` : '零点标定完成';
    addLog(msg, hasError ? 'error' : 'success');
    showNotification(hasError ? msg : '零点标定成功', hasError ? 'error' : 'success');
    updateZeroStatus(hasError ? msg : '标定成功');
    // 标定后自动刷新显示
    loadZeroOffset();
});

socket.on('zero:loaded', (data) => {
    if (data.offsets) {
        renderZeroOffsets(data.offsets);
        updateZeroLastUpdated();
        updateZeroStatus('已更新');
        addLog('已加载零点偏移并更新到界面', 'info');
    }
});

socket.on('speed:set', (data) => {
    if (data?.success) {
        const p = data?.params;
        if (p) {
            syncSpeedUI(p.velocity, p.accel, p.decel);
        }
        showNotification('速度设置成功', 'success');
        addLog('速度设置成功', 'success');
    }
});

socket.on('target:changed', (data) => {
    appState.target = data.target;
    updateTargetUI();
    addLog(`控制目标已切换: ${data.target}`, 'info');
});

// ==================== UI 更新函数 ====================

function updateConnectionStatus(connected) {
    const dot = document.querySelector('.status-dot');
    const text = document.querySelector('.status-text');

    if (connected) {
        dot.classList.add('connected');
        dot.classList.remove('error');
        text.textContent = '服务器已连接';
    } else {
        dot.classList.remove('connected');
        dot.classList.add('error');
        text.textContent = '服务器未连接';
    }
}

function updateUI() {
    updateArmsStatus();
    updateMonitor();
    updateJointSliders();
    updatePlaybackUI();
    updateDragTeachUI();
    updateTargetUI();
    updateCartesianDisplay();
    updateMotionUIState();
}

function restoreChannelInputs() {
    const leftEl = document.getElementById('left-channel');
    const rightEl = document.getElementById('right-channel');
    const savedLeft = localStorage.getItem('robot.left_channel');
    const savedRight = localStorage.getItem('robot.right_channel');

    if (leftEl && savedLeft) leftEl.value = savedLeft;
    if (rightEl && savedRight) rightEl.value = savedRight;

    if (leftEl && !savedLeft && appState.arms?.left?.can_channel) {
        leftEl.value = appState.arms.left.can_channel;
    }
    if (rightEl && !savedRight && appState.arms?.right?.can_channel) {
        rightEl.value = appState.arms.right.can_channel;
    }
}

function updateArmsStatus() {
    const leftTitle = document.getElementById('left-arm-title');
    const rightTitle = document.getElementById('right-arm-title');

    // 左臂状态
    const leftStatus = document.getElementById('left-arm-status');
    const leftState = appState.arms?.left;
    if (leftTitle) {
        const channel = leftState?.can_channel || document.getElementById('left-channel')?.value || '未知';
        leftTitle.textContent = `右臂 (${channel})`;
    }
    if (leftStatus && leftState) {
        leftStatus.className = 'arm-status';
        if (leftState.initialized) {
            leftStatus.classList.add('initialized');
            leftStatus.textContent = '已初始化';
        } else if (leftState.connected) {
            leftStatus.classList.add('connected');
            leftStatus.textContent = '已连接';
        } else {
            leftStatus.textContent = '未连接';
        }
    }

    // 右臂状态
    const rightStatus = document.getElementById('right-arm-status');
    const rightState = appState.arms?.right;
    if (rightTitle) {
        const channel = rightState?.can_channel || document.getElementById('right-channel')?.value || '未知';
        rightTitle.textContent = `左臂 (${channel})`;
    }
    if (rightStatus && rightState) {
        rightStatus.className = 'arm-status';
        if (rightState.initialized) {
            rightStatus.classList.add('initialized');
            rightStatus.textContent = '已初始化';
        } else if (rightState.connected) {
            rightStatus.classList.add('connected');
            rightStatus.textContent = '已连接';
        } else {
            rightStatus.textContent = '未连接';
        }
    }

    // 更新header状态
    const headerDot = document.querySelector('.status-dot');
    const anyConnected = leftState?.connected || rightState?.connected;
    if (anyConnected) {
        headerDot.classList.add('connected');
    } else {
        headerDot.classList.remove('connected');
    }
}

function updateMonitor() {
    updateArmMonitor('left');
    updateArmMonitor('right');
}

function updateArmMonitor(armId) {
    const tbody = document.getElementById(`${armId}-motors-tbody`);
    if (!tbody) return;

    const armState = appState.arms?.[armId];
    const motorsData = armState?.motors || {};
    const isConnected = !!(armState && armState.connected);

    const motorIds = armId === 'left' ? LEFT_MOTOR_IDS : RIGHT_MOTOR_IDS;
    let html = '';

    for (const mid of motorIds) {
        const motor = motorsData[mid] || {};
        const position = motor.position !== undefined ? motor.position.toFixed(4) : '-';
        const degrees = motor.position !== undefined ? (motor.position * 180 / Math.PI).toFixed(1) : '-';
        const limits = appState.joint_limits?.[armId]?.[mid];
        const minText = limits && typeof limits.min === 'number' ? limits.min.toFixed(2) : '-';
        const maxText = limits && typeof limits.max === 'number' ? limits.max.toFixed(2) : '-';
        
        // If not connected, show '未连接', else show enabled/disabled status
        const enabledText = isConnected ? (motor.enabled ? '使能' : '关闭') : '未连接';
        const enabledClass = isConnected ? (motor.enabled ? 'enabled' : 'disabled') : '';

        html += `
            <tr>
                <td>${mid}</td>
                <td class="position">${position}</td>
                <td>${degrees}°</td>
                <td>${minText}</td>
                <td>${maxText}</td>
                <td class="${enabledClass}">${enabledText}</td>
            </tr>
        `;
    }

    tbody.innerHTML = html;
}

function updateJointSliders() {
    const arms = ['left', 'right'];
    for (const armId of arms) {
        const motorIds = armId === 'left' ? LEFT_MOTOR_IDS : RIGHT_MOTOR_IDS;
        for (const mid of motorIds) {
            const sliderKey = `${armId}:${mid}`;

            // 跳过正在拖动的滑杆，防止状态更新覆盖用户输入
            if (draggingSliders.has(sliderKey)) {
                continue;
            }

            const slider = document.querySelector(`.joint-slider-small[data-arm="${armId}"][data-motor="${mid}"]`);
            if (!slider) continue;

            const limits = appState.joint_limits?.[armId]?.[mid];
            if (limits && typeof limits.min === 'number' && typeof limits.max === 'number') {
                slider.min = String(limits.min);
                slider.max = String(limits.max);
            }

            const pos = appState.arms?.[armId]?.motors?.[mid]?.relative_position;
            if (typeof pos === 'number') {
                const min = parseFloat(slider.min);
                const max = parseFloat(slider.max);
                const clamped = clamp(pos, isFinite(min) ? min : -3.14, isFinite(max) ? max : 3.14);
                slider.value = String(clamped);

                // 数值显示同步
                const labelId = armId === 'left' ? `pos-left-${mid}` : `pos-right-${mid}`;
                const labelEl = document.getElementById(labelId);
                if (labelEl) {
                    labelEl.textContent = clamped.toFixed(4);
                }

                // 更新偏差百分比指示器（距零点偏差占该方向量程的比例）
                const devId = `dev-${armId}-${mid}`;
                let devEl = document.getElementById(devId);
                if (!devEl) {
                    devEl = document.createElement('div');
                    devEl.id = devId;
                    devEl.className = 'joint-deviation dev-ok';
                    if (labelEl) labelEl.insertAdjacentElement('afterend', devEl);
                }
                if (devEl) {
                    const limitInDir = clamped >= 0
                        ? (isFinite(max) && max > 0 ? max : 3.14)
                        : (isFinite(min) && min < 0 ? Math.abs(min) : 3.14);
                    const devPct = limitInDir > 0 ? Math.round(Math.abs(clamped) / limitInDir * 100) : 0;
                    const level = devPct >= 85 ? 'dev-danger' : devPct >= 55 ? 'dev-warning' : 'dev-ok';
                    devEl.className = `joint-deviation ${level}`;
                    devEl.textContent = `${devPct}%`;
                    devEl.title = `距零点偏差: ${clamped.toFixed(3)} rad / ${devPct}% 量程`;
                }
            }
        }
    }
}

function updatePlaybackUI() {
    const playback = appState.playback || {};
    const progressFill = document.querySelector('.progress-fill');
    const progressText = document.querySelector('.progress-text');

    if (progressFill) {
        progressFill.style.width = `${playback.progress || 0}%`;
    }

    if (progressText) {
        if (playback.state === 'playing') {
            progressText.textContent = `${playback.current_point || 0}/${playback.total_points || 0} - ${Math.round(playback.progress || 0)}%`;
        } else if (playback.state === 'paused') {
            progressText.textContent = '已暂停';
        } else {
            progressText.textContent = '就绪';
        }
    }

    // 更新按钮状态
    const playBtn = document.getElementById('btn-play');
    const pauseBtn = document.getElementById('btn-pause');
    const stopBtn = document.getElementById('btn-stop');

    if (playBtn) playBtn.disabled = playback.state === 'playing';
    if (pauseBtn) pauseBtn.disabled = playback.state !== 'playing';
    if (stopBtn) stopBtn.disabled = playback.state === 'idle';
}

function updatePlaybackProgress(data) {
    appState.playback = {
        ...appState.playback,
        progress: data.progress,
        current_point: data.current,
        total_points: data.total
    };
    updatePlaybackUI();
}

function updateDragTeachUI() {
    const state = appState.drag_teach || {};
    const activeSession = state.active_session || null;
    const arms = state.arms || {};
    const enabled = !!state.enabled;

    let currentArmState = null;
    if (appState.target === 'left' || appState.target === 'right') {
        currentArmState = arms[appState.target] || null;
    } else {
        currentArmState = arms.left?.mode_enabled ? arms.left : (arms.right || null);
    }

    const statusText = currentArmState?.status || (enabled ? 'dragging' : 'idle');
    const sourceText = currentArmState?.controller_source || 'pp_fallback';
    const segmentText = activeSession?.name || currentArmState?.segment_name || '-';
    const pointCount = Array.isArray(activeSession?.points) ? activeSession.points.length : (currentArmState?.keyframe_count || 0);

    const statusEl = document.getElementById('drag-teach-status');
    const sourceEl = document.getElementById('drag-teach-source');
    const segmentEl = document.getElementById('drag-teach-current-segment');
    const countEl = document.getElementById('drag-teach-point-count');
    const hintEl = document.getElementById('drag-teach-session-hint');
    const inlineEl = document.getElementById('drag-teach-inline-status');

    if (statusEl) statusEl.textContent = statusText;
    if (sourceEl) sourceEl.textContent = sourceText;
    if (segmentEl) segmentEl.textContent = segmentText;
    if (countEl) countEl.textContent = String(pointCount);
    if (inlineEl) inlineEl.textContent = `状态: ${statusText}`;
    if (hintEl) {
        hintEl.textContent = activeSession
            ? `当前录制段: ${segmentText} / 模式: ${activeSession.sample_type || 'keyframe'}`
            : '可先启用拖拽示教，再开始录制段。';
    }
}

function updateTargetUI() {
    const options = document.querySelectorAll('.target-option');
    options.forEach(opt => {
        const value = opt.dataset.target;
        if (value === appState.target) {
            opt.classList.add('active');
        } else {
            opt.classList.remove('active');
        }
    });
}

// ==================== 操作函数 ====================

function connectArms() {
    if (armConnectInFlight) {
        addLog('机械臂正在连接中，请稍候...', 'warning');
        return;
    }
    const leftChannel = document.getElementById('left-channel')?.value || 'can0';
    const rightChannel = document.getElementById('right-channel')?.value || 'can1';

    localStorage.setItem('robot.left_channel', leftChannel);
    localStorage.setItem('robot.right_channel', rightChannel);
    armConnectInFlight = true;

    socket.emit('arm_connect', {
        arm_id: 'both',
        left_channel: leftChannel,
        right_channel: rightChannel
    });
    addLog(`正在连接机械臂... 右臂=${leftChannel}, 左臂=${rightChannel}`, 'info');
}

function disconnectArms() {
    socket.emit('arm_disconnect', { arm_id: 'both' });
}

function initArms() {
    const mode = parseInt(document.getElementById('init-mode')?.value || '6');
    socket.emit('arm_init', { mode: mode });
    addLog('正在启动使能机械臂...', 'info');
}

function deactivateArms() {
    socket.emit('arm_deactivate', {});
    addLog('正在去使能...', 'info');
}

function disableArms() {
    socket.emit('arm_disable', {});
    addLog('正在关闭电机...', 'info');
}

function emergencyStop() {
    socket.emit('emergency_stop', {});
    addLog('!!! 发送紧急停止命令 !!!', 'warning');
}

function goToZero() {
    socket.emit('go_to_zero', {});
    addLog('正在回零...', 'info');
}

function setTarget(target) {
    socket.emit('set_target', { target: target });
}

function loadTrajectories() {
    fetch('/api/trajectories')
        .then(res => res.json())
        .then(data => {
            appState.trajectories = data.trajectories || [];
            updateTrajectorySelect();
        })
        .catch(err => {
            console.error('加载轨迹列表失败:', err);
        });
}

function updateTrajectorySelect() {
    const select = document.getElementById('trajectory-select');
    if (!select) return;

    select.innerHTML = '<option value="">选择轨迹文件...</option>';
    for (const file of appState.trajectories) {
        select.innerHTML += `<option value="${file}">${file}</option>`;
    }
}

function playTrajectory() {
    const filename = document.getElementById('trajectory-select')?.value;
    if (!filename) {
        showNotification('请先选择轨迹文件', 'warning');
        return;
    }
    const loop = document.getElementById('loop-toggle')?.checked || false;
    socket.emit('trajectory_play', { filename: filename, sync: true, loop: loop });
}

function pauseTrajectory() {
    socket.emit('trajectory_pause', {});
}

function resumeTrajectory() {
    if (appState.playback?.state === 'paused') {
        socket.emit('trajectory_resume', {});
    } else {
        playTrajectory();
    }
}

function stopTrajectory() {
    socket.emit('trajectory_stop', {});
}

function playPreset(presetId) {
    const filename = `preset_${presetId.toLowerCase()}.json`;
    socket.emit('trajectory_play', { filename: filename, sync: true });
    addLog(`执行预设动作: ${presetId}`, 'info');

    // 高亮当前按钮
    document.querySelectorAll('.preset-btn').forEach(btn => btn.classList.remove('active'));
    document.querySelector(`[data-preset="${presetId}"]`)?.classList.add('active');
}

function setSpeed(preset) {
    socket.emit('set_speed', { preset: preset });
    addLog(`设置速度: ${preset}`, 'info');
}

function setCustomSpeed() {
    const velocity = parseFloat(document.getElementById('speed-velocity')?.value || '1.0');
    const accel = parseFloat(document.getElementById('speed-accel')?.value || '1.0');
    const decel = parseFloat(document.getElementById('speed-decel')?.value || '1.0');

    socket.emit('set_speed', { velocity, accel, decel });
    addLog(`设置自定义速度: v=${velocity}, a=${accel}, d=${decel}`, 'info');
}

function getSelectedArmIdForDragTeach() {
    return appState.target === 'left' || appState.target === 'right'
        ? appState.target
        : null;
}

function enableFreedrive() {
    socket.emit('drag_teach_enable', {
        arm_id: getSelectedArmIdForDragTeach(),
        controller_source: 'soft_compliance',
        soft_position_kp: 6.0,
        soft_velocity_kp: 0.08,
        soft_velocity_ki: 0.0,
        soft_velocity_limit: 0.35,
        soft_acceleration: 1.0
    });
    addLog('启用拖拽示教（低阻尼顺从）模式', 'info');
}

function disableFreedrive() {
    socket.emit('drag_teach_disable', {
        arm_id: getSelectedArmIdForDragTeach()
    });
    addLog('退出拖拽示教模式', 'info');
}

function dragTeachStartSegment() {
    const name = document.getElementById('drag-segment-name')?.value || `segment_${Date.now()}`;
    const description = document.getElementById('drag-segment-desc')?.value || '';
    const sampleType = document.getElementById('drag-segment-sample-type')?.value || 'keyframe';
    socket.emit('drag_teach_start_segment', {
        arm_id: getSelectedArmIdForDragTeach(),
        name,
        description,
        sample_type: sampleType
    });
}

function dragTeachRecordKeyframe() {
    const name = document.getElementById('traj-point-name')?.value || '';
    const delay = parseFloat(document.getElementById('drag-keyframe-delay')?.value || '1.0');
    socket.emit('drag_teach_record_keyframe', {
        arm_id: getSelectedArmIdForDragTeach(),
        name,
        delay
    });
}

function dragTeachRemoveLastPoint() {
    socket.emit('drag_teach_remove_last_point', {});
}

function dragTeachStopSegment() {
    socket.emit('drag_teach_stop_segment', { save: true });
}

function dragTeachCancelSegment() {
    socket.emit('drag_teach_cancel_segment', {});
}

function dragTeachListSegments() {
    socket.emit('drag_teach_list_segments', {});
}

function jogMotor(armId, motorId, direction) {
    const step = parseFloat(document.getElementById('jog-step')?.value || '0.1');
    const currentPos = appState.arms?.[armId]?.motors?.[motorId]?.position || 0;
    let dir = direction;
    if (armId === 'right' && RIGHT_INVERT_MOTORS.has(motorId)) {
        dir = -dir;
    }
    const newPos = currentPos + (dir * step);

    const limits = appState.joint_limits?.[armId]?.[motorId];
    if (limits) {
        const min = Number(limits.min);
        const max = Number(limits.max);
        if (isFinite(min) && isFinite(max)) {
            if (newPos < min || newPos > max) {
                showNotification(`关节 ${motorId} 超出安全范围 (${min.toFixed(2)} ~ ${max.toFixed(2)} rad)，已阻止操作。`, 'warning');
                addLog(`关节 ${armId}.${motorId} 尝试越界到 ${newPos.toFixed(4)} rad，已阻止。`, 'warning');
                return;
            }
            const margin = (max - min) * 0.05;
            if (newPos - min < margin || max - newPos < margin) {
                showNotification(`关节 ${motorId} 已接近软限位，请谨慎操作。`, 'warning');
            }
        }
    }

    // 使用平滑插值移动
    socket.emit('smooth_manual_move', {
        arm_id: armId,
        motor_id: motorId,
        position: newPos,
        duration_ms: 300
    });
}

function zeroMotor(armId, motorId) {
    socket.emit('manual_set_position_offset', {
        arm_id: armId,
        motor_id: motorId,
        position: 0
    });
}

function calibrateZero() {
    socket.emit('zero_calibrate', {});
    addLog('正在标定零点...', 'info');
    updateZeroStatus('标定中...');
}

function loadZeroOffset() {
    socket.emit('zero_load', {});
    addLog('正在查看零点偏移...', 'info');
}

function updateZeroStatus(text) {
    const el = document.getElementById('zero-calib-status');
    if (!el) return;
    el.textContent = text || '';
}

function updateZeroLastUpdated() {
    const el = document.getElementById('zero-last-updated');
    if (!el) return;
    el.textContent = new Date().toLocaleString();
}

function renderZeroOffsets(offsetsByArm) {
    const leftEl = document.getElementById('zero-left-list');
    const rightEl = document.getElementById('zero-right-list');
    if (!leftEl || !rightEl) return;

    const fmt = (obj) => {
        if (!obj || Object.keys(obj).length === 0) return '无数据';
        const lines = Object.entries(obj)
            .sort((a, b) => Number(a[0]) - Number(b[0]))
            .map(([mid, off]) => `电机 ${mid}: ${Number(off).toFixed(4)} rad`);
        return lines.join('\n');
    };

    leftEl.textContent = fmt(offsetsByArm.left);
    rightEl.textContent = fmt(offsetsByArm.right);
}

// ==================== 速度滑杆 ====================

let _speedSliderDebounce = null;

function initSpeedSlider() {
    const slider = document.getElementById('speed-slider');
    const velInput = document.getElementById('speed-velocity');
    if (!slider || !velInput) return;

    // 暂时使用页面已有上限（未知电机上限时不强行猜）
    const max = parseFloat(slider.max || velInput.max || '2.0');
    slider.min = '0';
    slider.max = String(isFinite(max) ? max : 2.0);
    slider.step = '0.01';

    const maxEl = document.getElementById('speed-max');
    const minEl = document.getElementById('speed-min');
    if (maxEl) maxEl.textContent = slider.max;
    if (minEl) minEl.textContent = slider.min;

    // 初始同步
    const initV = clamp(parseFloat(velInput.value || '1.0'), parseFloat(slider.min), parseFloat(slider.max));
    slider.value = String(initV);
    updateSpeedSliderValue(initV);

    slider.addEventListener('input', () => {
        const v = clamp(parseFloat(slider.value), parseFloat(slider.min), parseFloat(slider.max));
        velInput.value = v.toFixed(2);
        updateSpeedSliderValue(v);

        // 200ms 去抖发送，避免拖动时刷屏
        if (_speedSliderDebounce) clearTimeout(_speedSliderDebounce);
        _speedSliderDebounce = setTimeout(() => {
            setCustomSpeed();
        }, 200);
    });

    velInput.addEventListener('change', () => {
        const v = clamp(parseFloat(velInput.value || '0'), parseFloat(slider.min), parseFloat(slider.max));
        velInput.value = v.toFixed(2);
        slider.value = String(v);
        updateSpeedSliderValue(v);
    });
}

function updateSpeedSliderValue(v) {
    const el = document.getElementById('speed-slider-value');
    const cur = document.getElementById('speed-current');
    if (el) el.textContent = Number(v).toFixed(2);
    if (cur) cur.textContent = Number(v).toFixed(2);
}

function syncSpeedUI(velocity, accel, decel) {
    const v = Number(velocity);
    const a = Number(accel);
    const d = Number(decel);
    const velInput = document.getElementById('speed-velocity');
    const accelInput = document.getElementById('speed-accel');
    const decelInput = document.getElementById('speed-decel');
    const slider = document.getElementById('speed-slider');

    if (velInput && isFinite(v)) velInput.value = v.toFixed(2);
    if (accelInput && isFinite(a)) accelInput.value = a.toFixed(2);
    if (decelInput && isFinite(d)) decelInput.value = d.toFixed(2);

    if (slider && isFinite(v)) {
        const min = parseFloat(slider.min || '0');
        const max = parseFloat(slider.max || '2');
        const clamped = clamp(v, min, max);
        slider.value = String(clamped);
        updateSpeedSliderValue(clamped);
    }
}

function clamp(x, min, max) {
    if (!isFinite(x)) return min;
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

function recordPoint() {
    const name = document.getElementById('point-name')?.value || '';
    socket.emit('teaching_record', { name: name });
    addLog(`记录位置点: ${name || '(unnamed)'}`, 'info');
}

// ==================== 轨迹示教系统 ====================

let _trajFileListMode = null; // 'load' | 'delete' | null

function trajectoryNewPrompt() {
    const name = prompt('请输入轨迹名称:', `轨迹_${Date.now()}`);
    if (!name) return;
    const desc = prompt('描述 (可留空):', '') || '';
    socket.emit('trajectory_new', { name: name, description: desc });
    addLog(`新建轨迹: ${name}`, 'info');
}

function trajectoryListAndLoad() {
    _trajFileListMode = 'load';
    socket.emit('trajectory_list');
}

function trajectoryListAndDelete() {
    _trajFileListMode = 'delete';
    socket.emit('trajectory_list');
}

function trajectoryRefreshList() {
    _trajFileListMode = _trajFileListMode || 'load';
    socket.emit('trajectory_list');
}

function trajectoryAddPoint() {
    const name = document.getElementById('traj-point-name')?.value || '';
    const delay = parseFloat(document.getElementById('traj-point-delay')?.value || '1.0');
    socket.emit('trajectory_add_point', { name: name, delay: delay });
    // 清空名称输入框
    const nameEl = document.getElementById('traj-point-name');
    if (nameEl) nameEl.value = '';
}

function trajectoryRemovePoint(index) {
    socket.emit('trajectory_remove_point', { index: index });
}

function trajectoryMovePoint(fromIdx, toIdx) {
    socket.emit('trajectory_reorder', { from_index: fromIdx, to_index: toIdx });
}

function trajectorySave() {
    // 同步元数据到后端
    const name = document.getElementById('traj-edit-name')?.value || '';
    const desc = document.getElementById('traj-edit-desc')?.value || '';
    const loop = document.getElementById('traj-edit-loop')?.checked || false;
    const speed = parseFloat(document.getElementById('traj-edit-speed')?.value || '1.0');
    socket.emit('trajectory_update_meta', { name, description: desc, loop, speed_multiplier: speed });
    // 稍延后保存
    setTimeout(() => {
        socket.emit('trajectory_save');
    }, 100);
}

function trajectoryPreview() {
    // 先保存, 再播放
    trajectorySave();
    setTimeout(() => {
        // 后端保存后会发送 trajectory:saved
    }, 300);
}

function trajectoryCloseEdit() {
    socket.emit('trajectory_close_edit');
}

function renderTrajectoryEditor(data) {
    const editor = document.getElementById('traj-editor');
    if (!editor) return;

    if (!data || !data.editing) {
        editor.classList.add('hidden');
        return;
    }

    editor.classList.remove('hidden');
    const traj = data.trajectory;
    if (!traj) return;

    document.getElementById('traj-editor-title').textContent = `编辑: ${traj.name}`;
    document.getElementById('traj-edit-name').value = traj.name || '';
    document.getElementById('traj-edit-desc').value = traj.description || '';
    document.getElementById('traj-edit-loop').checked = traj.loop || false;
    document.getElementById('traj-edit-speed').value = traj.speed_multiplier || 1.0;

    const pts = traj.points || [];
    document.getElementById('traj-point-count').textContent = `${pts.length} 个点位`;
    renderPointList(pts);
}

function renderPointList(points) {
    const container = document.getElementById('traj-point-list');
    if (!container) return;

    if (!points || points.length === 0) {
        container.innerHTML = '<div class="traj-empty-hint">暂无点位, 请使用下方按钮记录</div>';
        return;
    }

    container.innerHTML = points.map((pt, i) => `
        <div class="traj-point-item">
            <span class="traj-point-idx">#${i + 1}</span>
            <span class="traj-point-name">${escapeHtml(pt.name || `point_${i + 1}`)}</span>
            <span class="traj-point-delay">${(pt.delay || 0).toFixed(1)}s</span>
            <div class="traj-point-actions">
                <button onclick="trajectoryMovePoint(${i}, ${Math.max(0, i - 1)})" ${i === 0 ? 'disabled' : ''}>↑</button>
                <button onclick="trajectoryMovePoint(${i}, ${Math.min(points.length - 1, i + 1)})" ${i === points.length - 1 ? 'disabled' : ''}>↓</button>
                <button class="btn-del" onclick="trajectoryRemovePoint(${i})">✕</button>
            </div>
        </div>
    `).join('');
}

function renderFileList(files, mode) {
    const container = document.getElementById('traj-file-list');
    if (!container) return;

    if (!files || files.length === 0) {
        container.innerHTML = '<div class="traj-empty-hint">没有轨迹文件</div>';
        container.classList.remove('hidden');
        return;
    }

    const actionLabel = mode === 'delete' ? '🗑️ 删除' : '📂 加载';
    const actionClass = mode === 'delete' ? 'btn btn-outline btn-sm' : 'btn btn-success btn-sm';

    container.innerHTML = files.map(f => `
        <div class="traj-file-item">
            <div class="traj-file-info">
                <div class="traj-file-name">${escapeHtml(f.name)}</div>
                <div class="traj-file-meta">${f.filename} · ${f.points_count} 个点位</div>
            </div>
            <button class="${actionClass}" onclick="trajFileAction('${escapeHtml(f.filename)}', '${mode}')">${actionLabel}</button>
        </div>
    `).join('');
    container.classList.remove('hidden');
}

function trajFileAction(filename, mode) {
    if (mode === 'delete') {
        if (!confirm(`确定删除轨迹 ${filename} ？`)) return;
        socket.emit('trajectory_delete_file', { filename: filename });
    } else {
        socket.emit('trajectory_load_edit', { filename: filename });
        // 隐藏文件列表
        const el = document.getElementById('traj-file-list');
        if (el) el.classList.add('hidden');
    }
}

// Socket.IO 轨迹示教监听
socket.on('trajectory:edit_state', (data) => {
    renderTrajectoryEditor(data);
});

socket.on('trajectory:list', (data) => {
    renderFileList(data.files, _trajFileListMode);
});

socket.on('trajectory:saved', (data) => {
    addLog(`轨迹已保存: ${data.filename}`, 'success');
    showNotification(`轨迹 "${data.name}" 已保存`, 'success');
});

socket.on('trajectory:deleted', (data) => {
    addLog(`轨迹已删除: ${data.filename}`, 'info');
    showNotification(`已删除 ${data.filename}`, 'info');
    // 刷新列表
    socket.emit('trajectory_list');
});

socket.on('trajectory:point_added', (data) => {
    addLog(`记录点位 #${data.index + 1}: ${data.name}`, 'success');
    showNotification(`点位 "${data.name}" 已记录`, 'success');
});

socket.on('trajectory:created', (data) => {
    addLog(`新建轨迹: ${data.name}`, 'success');
});

socket.on('trajectory:loaded', (data) => {
    addLog(`已加载轨迹: ${data.name}`, 'success');
});

// ==================== 笛卡尔控制 ====================

// 防止滑杆拖动时被状态更新覆盖
let _cartSliderDragging = { x: false, y: false, z: false };

function updateCartesianDisplay() {
    const cart = appState.cartesian;
    if (!cart) return;

    // 取当前选中臂更新反馈
    const selArm = document.getElementById('cartesian-arm')?.value || 'left';

    for (const armId of ['left', 'right']) {
        const data = cart[armId];
        if (!data) continue;

        const xEl = document.getElementById(`cart-${armId}-x`);
        const yEl = document.getElementById(`cart-${armId}-y`);
        const zEl = document.getElementById(`cart-${armId}-z`);

        if (xEl) xEl.textContent = (data.x !== undefined ? data.x.toFixed(2) : '---');
        if (yEl) yEl.textContent = (data.y !== undefined ? data.y.toFixed(2) : '---');
        if (zEl) zEl.textContent = (data.z !== undefined ? data.z.toFixed(2) : '---');
    }

    const selData = cart[selArm];
    if (selData) {
        const currentPose = {
            x: Number(selData.x || 0) / 1000,
            y: Number(selData.y || 0) / 1000,
            z: Number(selData.z || 0) / 1000,
            rx: Number(selData.roll || 0) * Math.PI / 180,
            ry: Number(selData.pitch || 0) * Math.PI / 180,
            rz: Number(selData.yaw || 0) * Math.PI / 180
        };
        renderMotionCurrentPose(currentPose);
    }
    updateCartesianLiveUI();
}

function updateCartesianLiveUI() {
    const live = appState.cartesian_live;
    const bothEl = document.getElementById('motion-live-both');
    const metaEl = document.getElementById('motion-live-meta');
    if (!bothEl || !metaEl) return;
    if (!live || !live.arms) {
        bothEl.textContent = '-';
        metaEl.textContent = '-';
        return;
    }

    const fmtArm = (armId) => {
        const a = live.arms?.[armId];
        if (!a || !a.connected) return `${armId}: N/A`;
        return `${armId}: x:${a.x_mm.toFixed(1)} y:${a.y_mm.toFixed(1)} z:${a.z_mm.toFixed(1)} | rx:${a.rx_deg.toFixed(1)} ry:${a.ry_deg.toFixed(1)} rz:${a.rz_deg.toFixed(1)}`;
    };

    bothEl.textContent = `${fmtArm('left')} || ${fmtArm('right')}`;
    const ts = Number(live.server_ts_ms || 0);
    const playback = live.playback_state || 'unknown';
    metaEl.textContent = `${ts ? new Date(ts).toLocaleTimeString() : '-'} | playback=${playback}`;

    // 当前Pose优先显示实时流中“当前选中臂”的值。
    const selArm = document.getElementById('cartesian-arm')?.value || 'left';
    const sel = live.arms?.[selArm];
    if (sel && sel.connected) {
        renderMotionCurrentPose({
            x: Number(sel.x_mm) / 1000.0,
            y: Number(sel.y_mm) / 1000.0,
            z: Number(sel.z_mm) / 1000.0,
            rx: degToRad(Number(sel.rx_deg)),
            ry: degToRad(Number(sel.ry_deg)),
            rz: degToRad(Number(sel.rz_deg)),
        });
    }
}

let _cartSliderTimer = null;

function cartesianSliderMove(axis) {
    const slider = document.getElementById(`cart-slider-${axis}`);
    if (!slider) return;
    const val = parseFloat(slider.value);
    const input = document.getElementById(`cart-input-${axis}`);
    if (input) input.value = val.toFixed(1);

    // debounce: 发送平滑笛卡尔移动
    clearTimeout(_cartSliderTimer);
    _cartSliderTimer = setTimeout(() => {
        const armId = document.getElementById('cartesian-arm')?.value || 'left';
        const x = parseFloat(document.getElementById('cart-slider-x')?.value || '0');
        const y = parseFloat(document.getElementById('cart-slider-y')?.value || '0');
        const z = parseFloat(document.getElementById('cart-slider-z')?.value || '0');
        socket.emit('smooth_cartesian_move', {
            arm_id: armId,
            x: x, y: y, z: z,
            duration_ms: 400
        });
        addLog(`笛卡尔滑杆: ${armId} → (${x}, ${y}, ${z}) mm`, 'info');
    }, 120);
}

function cartesianInputMove(axis) {
    const input = document.getElementById(`cart-input-${axis}`);
    if (!input) return;
    const val = parseFloat(input.value);
    const slider = document.getElementById(`cart-slider-${axis}`);
    if (slider) slider.value = val;

    const armId = document.getElementById('cartesian-arm')?.value || 'left';
    const x = parseFloat(document.getElementById('cart-input-x')?.value || '0');
    const y = parseFloat(document.getElementById('cart-input-y')?.value || '0');
    const z = parseFloat(document.getElementById('cart-input-z')?.value || '0');

    socket.emit('smooth_cartesian_move', {
        arm_id: armId,
        x: x, y: y, z: z,
        duration_ms: 500
    });
    addLog(`笛卡尔输入: ${armId} ${axis.toUpperCase()}=${val}mm`, 'info');
}

function bindCartesianSliderEvents() {
    for (const axis of ['x', 'y', 'z']) {
        const slider = document.getElementById(`cart-slider-${axis}`);
        if (!slider) continue;
        slider.addEventListener('mousedown', () => { _cartSliderDragging[axis] = true; });
        slider.addEventListener('touchstart', () => { _cartSliderDragging[axis] = true; });
        slider.addEventListener('mouseup', () => { _cartSliderDragging[axis] = false; });
        slider.addEventListener('touchend', () => { _cartSliderDragging[axis] = false; });
        slider.addEventListener('input', () => cartesianSliderMove(axis));
    }
}

function cartesianJog(axis, direction) {
    executeJogOnce(axis, direction);
}

function cartesianMoveTo() {
    executeMotion();
}

function degToRad(deg) {
    return Number(deg) * Math.PI / 180;
}

function radToDeg(rad) {
    return Number(rad) * 180 / Math.PI;
}

function genCommandId() {
    if (window.crypto && window.crypto.randomUUID) return window.crypto.randomUUID();
    return `cmd-${Date.now()}-${Math.floor(Math.random() * 100000)}`;
}

function getMotionConstraints() {
    return {
        max_velocity: parseFloat(document.getElementById('motion-max-vel')?.value || '0.05'),
        max_acceleration: parseFloat(document.getElementById('motion-max-acc')?.value || '0.10'),
        max_angular_velocity: parseFloat(document.getElementById('motion-max-ang-vel')?.value || '0.30'),
        max_angular_acceleration: parseFloat(document.getElementById('motion-max-ang-acc')?.value || '0.10')
    };
}

function getTargetPoseFromUI() {
    return {
        x: parseFloat(document.getElementById('cart-target-x')?.value || '0') / 1000,
        y: parseFloat(document.getElementById('cart-target-y')?.value || '0') / 1000,
        z: parseFloat(document.getElementById('cart-target-z')?.value || '0') / 1000,
        rx: degToRad(document.getElementById('cart-target-rx')?.value || '0'),
        ry: degToRad(document.getElementById('cart-target-ry')?.value || '0'),
        rz: degToRad(document.getElementById('cart-target-rz')?.value || '0')
    };
}

function renderMotionCurrentPose(pose) {
    const el = document.getElementById('motion-current-pose');
    if (!el || !pose) return;
    el.textContent = `x:${(pose.x * 1000).toFixed(1)} y:${(pose.y * 1000).toFixed(1)} z:${(pose.z * 1000).toFixed(1)} | rx:${radToDeg(pose.rx).toFixed(1)} ry:${radToDeg(pose.ry).toFixed(1)} rz:${radToDeg(pose.rz).toFixed(1)}`;
}

function setMotionMode(mode) {
    motionState.mode = mode;
    document.getElementById('motion-mode-ptp')?.classList.toggle('active', mode === 'CARTESIAN_PTP');
    document.getElementById('motion-mode-linear')?.classList.toggle('active', mode === 'CARTESIAN_LINEAR');
    document.getElementById('motion-mode-jog')?.classList.toggle('active', mode === 'CARTESIAN_JOG');
    applyMotionModeUI();
    updateMotionUIState();
}

function applyMotionModeUI() {
    const mode = motionState.mode;
    const meta = MOTION_MODE_META[mode] || MOTION_MODE_META.CARTESIAN_PTP;
    const desc = document.getElementById('motion-mode-desc');
    const execBtn = document.getElementById('btn-motion-exec');
    const poseSection = document.getElementById('motion-pose-section');
    const jogTitle = document.getElementById('motion-jog-title');
    const jogControls = document.getElementById('motion-jog-controls');
    const jogGrid = document.getElementById('motion-jog-grid');

    if (desc) desc.textContent = meta.desc;
    if (execBtn) execBtn.textContent = meta.execText;

    const isJog = mode === 'CARTESIAN_JOG';
    poseSection?.classList.toggle('mode-hidden', isJog);
    jogTitle?.classList.toggle('mode-hidden', !isJog);
    jogControls?.classList.toggle('mode-hidden', !isJog);
    jogGrid?.classList.toggle('mode-hidden', !isJog);
}

function resetMotionConstraints() {
    document.getElementById('motion-max-vel').value = '0.05';
    document.getElementById('motion-max-acc').value = '0.10';
    document.getElementById('motion-max-ang-vel').value = '0.30';
    document.getElementById('motion-max-ang-acc').value = '0.10';
}

function executeMotion() {
    if (motionState.activeCommandId) {
        showNotification('执行中禁止重复提交', 'warning');
        return;
    }
    if (motionState.mode === 'CARTESIAN_JOG') {
        showNotification('JOG 模式请使用下方点动按钮', 'info');
        return;
    }
    const armId = document.getElementById('cartesian-arm')?.value || 'left';
    const frame = document.getElementById('motion-frame')?.value || 'BASE';
    const commandId = genCommandId();
    const targetPose = getTargetPoseFromUI();
    const payload = {
        command_id: commandId,
        arm_id: armId,
        motion_type: motionState.mode,
        frame: frame,
        target: { pose: targetPose },
        constraints: getMotionConstraints(),
        options: { blocking: false, timeout_ms: 30000, blend_radius: 0.0 },
        client_ts: Date.now()
    };
    motionState.lifecycle = 'pending';
    motionState.activeArmId = armId;
    document.getElementById('motion-target-pose').textContent =
        `x:${(targetPose.x * 1000).toFixed(1)} y:${(targetPose.y * 1000).toFixed(1)} z:${(targetPose.z * 1000).toFixed(1)} | rx:${radToDeg(targetPose.rx).toFixed(1)} ry:${radToDeg(targetPose.ry).toFixed(1)} rz:${radToDeg(targetPose.rz).toFixed(1)}`;
    socket.emit('motion:execute', payload);
    addLog(`发送命令: ${payload.motion_type} arm=${armId} frame=${frame}`, 'info');
    updateMotionUIState();
}

function executeJogOnce(axis, direction) {
    if (motionState.mode !== 'CARTESIAN_JOG') {
        return;
    }
    const armId = document.getElementById('cartesian-arm')?.value || 'left';
    const frame = document.getElementById('motion-frame')?.value || 'BASE';
    const mmStep = parseFloat(document.getElementById('cartesian-step-mm')?.value || '5');
    const degStep = parseFloat(document.getElementById('cartesian-step-deg')?.value || '1');
    const delta = {
        dx: 0, dy: 0, dz: 0, drx: 0, dry: 0, drz: 0
    };
    if (axis === 'x') delta.dx = direction * mmStep / 1000;
    if (axis === 'y') delta.dy = direction * mmStep / 1000;
    if (axis === 'z') delta.dz = direction * mmStep / 1000;
    if (axis === 'rx') delta.drx = direction * degToRad(degStep);
    if (axis === 'ry') delta.dry = direction * degToRad(degStep);
    if (axis === 'rz') delta.drz = direction * degToRad(degStep);
    socket.emit('motion:execute', {
        command_id: genCommandId(),
        arm_id: armId,
        motion_type: 'CARTESIAN_JOG',
        frame: frame,
        target: { delta: delta },
        constraints: getMotionConstraints(),
        options: { blocking: false },
        client_ts: Date.now()
    });
}

function pauseMotion() {
    socket.emit('motion:pause', { command_id: motionState.activeCommandId, arm_id: motionState.activeArmId || 'left' });
}

function resumeMotion() {
    socket.emit('motion:resume', { command_id: motionState.activeCommandId, arm_id: motionState.activeArmId || 'left' });
}

function cancelMotion() {
    socket.emit('motion:cancel', { command_id: motionState.activeCommandId, arm_id: motionState.activeArmId || 'left' });
}

function updateMotionUIState() {
    const state = motionState.lifecycle || 'idle';
    const indicator = document.getElementById('motion-state-indicator');
    if (indicator) indicator.textContent = state.toUpperCase();
    const running = ['accepted', 'planning', 'running', 'paused', 'pending'].includes(state);
    document.querySelectorAll('#tab-cartesian input, #tab-cartesian select').forEach((el) => {
        if (['btn-motion-pause', 'btn-motion-resume', 'btn-motion-cancel', 'btn-motion-exec'].includes(el.id)) return;
        if (state === 'running' || state === 'accepted' || state === 'planning') el.disabled = true;
        if (!running) el.disabled = false;
    });
    const execBtn = document.getElementById('btn-motion-exec');
    const pauseBtn = document.getElementById('btn-motion-pause');
    const resumeBtn = document.getElementById('btn-motion-resume');
    const cancelBtn = document.getElementById('btn-motion-cancel');
    if (execBtn) {
        const isJog = motionState.mode === 'CARTESIAN_JOG';
        execBtn.disabled = running || isJog;
        execBtn.title = isJog ? 'JOG 模式请使用下方点动按钮' : '';
    }
    if (pauseBtn) pauseBtn.disabled = !(state === 'running');
    if (resumeBtn) resumeBtn.disabled = !(state === 'paused');
    if (cancelBtn) cancelBtn.disabled = !(state === 'running' || state === 'paused' || state === 'planning' || state === 'accepted');

    document.querySelectorAll('#motion-mode-ptp, #motion-mode-linear, #motion-mode-jog').forEach((btn) => {
        btn.disabled = running;
    });
}

function bindJogHoldEvents() {
    const buttons = document.querySelectorAll('.jog-hold-btn');
    buttons.forEach((btn) => {
        const axis = btn.dataset.axis;
        const sign = parseInt(btn.dataset.sign || '1', 10);
        const start = () => {
            if (motionState.mode !== 'CARTESIAN_JOG') {
                showNotification('请先切换到 JOG 模式', 'info');
                return;
            }
            if (motionState.jogTimer) clearInterval(motionState.jogTimer);
            executeJogOnce(axis, sign);
            motionState.jogTimer = setInterval(() => executeJogOnce(axis, sign), 180);
        };
        const stop = () => {
            if (motionState.jogTimer) {
                clearInterval(motionState.jogTimer);
                motionState.jogTimer = null;
            }
        };
        btn.addEventListener('mousedown', start);
        btn.addEventListener('mouseup', stop);
        btn.addEventListener('mouseleave', stop);
        btn.addEventListener('touchstart', (e) => { e.preventDefault(); start(); }, { passive: false });
        btn.addEventListener('touchend', stop);
    });
}

socket.on('cartesian:moved', (data) => {
    addLog(`笛卡尔移动完成: ${data.arm_id} 误差=${data.error_mm?.toFixed(3)}mm`, 'success');
});

socket.on('cartesian:jogged', (data) => {
    const np = data.new_pos;
    addLog(`点动完成: ${data.arm_id} ${data.axis}${data.delta > 0 ? '+' : ''}${(data.delta * 1000).toFixed(1)}mm → (${(np.x * 1000).toFixed(1)}, ${(np.y * 1000).toFixed(1)}, ${(np.z * 1000).toFixed(1)})mm`, 'success');
});

socket.on('cartesian:result', (data) => {
    if (data.error) {
        addLog(`FK查询失败: ${data.error}`, 'error');
    } else {
        addLog(`FK: ${data.arm_id} (${(data.x * 1000).toFixed(1)}, ${(data.y * 1000).toFixed(1)}, ${(data.z * 1000).toFixed(1)}) mm`, 'info');
    }
});

function loadZeroOffsetFromFile() {
    fetch('/api/zero_offsets')
        .then(res => res.json())
        .then(data => {
            if (!data) return;
            const left = data.left || {};
            const right = data.right || {};
            renderZeroOffsets({ left, right });
            updateZeroLastUpdated();
            updateZeroStatus('已从文件加载');
            addLog('已从本地文件加载零点偏移', 'info');
        })
        .catch(err => {
            console.warn('从文件加载零点偏移失败:', err);
        });
}

function bindJointSliderEvents() {
    document.querySelectorAll('.joint-slider-small').forEach(slider => {
        const armId = slider.dataset.arm;
        const motorId = Number(slider.dataset.motor);
        if (!armId || !motorId) return;

        const sliderKey = `${armId}:${motorId}`;

        // 拖动开始
        slider.addEventListener('mousedown', () => {
            draggingSliders.add(sliderKey);
        });

        // 拖动结束
        slider.addEventListener('mouseup', () => {
            draggingSliders.delete(sliderKey);
            const finalPos = parseFloat(slider.value);
            if (isFinite(finalPos)) {
                sendPositionCommand(armId, motorId, finalPos);
            }
        });

        // 触摸设备支持
        slider.addEventListener('touchstart', () => {
            draggingSliders.add(sliderKey);
        });

        slider.addEventListener('touchend', () => {
            draggingSliders.delete(sliderKey);
            const finalPos = parseFloat(slider.value);
            if (isFinite(finalPos)) {
                sendPositionCommand(armId, motorId, finalPos);
            }
        });

        // 拖动过程中（防抖）
        slider.addEventListener('input', () => {
            const targetPos = parseFloat(slider.value);
            if (!isFinite(targetPos)) return;

            // 软限位检查
            const limits = appState.joint_limits?.[armId]?.[motorId];
            if (limits) {
                const min = Number(limits.min);
                const max = Number(limits.max);
                if (isFinite(min) && isFinite(max)) {
                    if (targetPos < min || targetPos > max) {
                        showNotification(`关节 ${motorId} 超出安全范围 (${min.toFixed(2)} ~ ${max.toFixed(2)} rad)，已阻止操作。`, 'warning');
                        return;
                    }
                }
            }

            // 更新显示
            const labelId = armId === 'left' ? `pos-left-${motorId}` : `pos-right-${motorId}`;
            const labelEl = document.getElementById(labelId);
            if (labelEl) {
                labelEl.textContent = targetPos.toFixed(4);
            }

            // 防抖：拖动过程中延迟发送命令
            if (draggingSliders.has(sliderKey)) {
                debouncedPositionCommand(armId, motorId, targetPos);
            } else {
                sendPositionCommand(armId, motorId, targetPos);
            }
        });
    });
}

// 防抖发送位置命令（拖动过程中每100ms最多发送一次）
function debouncedPositionCommand(armId, motorId, position) {
    const key = `${armId}:${motorId}`;

    // 清除之前的定时器
    if (pendingPositionCommands.has(key)) {
        clearTimeout(pendingPositionCommands.get(key).timeoutId);
    }

    // 设置新的定时器
    const timeoutId = setTimeout(() => {
        sendPositionCommand(armId, motorId, position);
        pendingPositionCommands.delete(key);
    }, 100);

    pendingPositionCommands.set(key, { position, timeoutId });
}

// 发送位置命令
function sendPositionCommand(armId, motorId, position) {
    // 滑杆显示的是相对零点角度，应走 offset 接口，避免绝对/相对语义错位。
    socket.emit('manual_set_position_offset', {
        arm_id: armId,
        motor_id: motorId,
        position: position
    });
}

// ==================== Tab 切换 ====================

function switchTab(tabId) {
    // 切换按钮状态
    document.querySelectorAll('.tab-btn[data-tab]').forEach(btn => {
        btn.classList.toggle('active', btn.dataset.tab === tabId);
    });

    // 切换内容
    document.querySelectorAll('.tab-content').forEach(content => {
        content.classList.toggle('active', content.id === `tab-${tabId}`);
    });
}

// ==================== 日志 ====================

function addLog(message, type = 'info') {
    const container = document.getElementById('log-container');
    if (!container) return;

    const time = new Date().toLocaleTimeString();
    const entry = document.createElement('div');
    entry.className = `log-entry ${type}`;
    entry.innerHTML = `<span class="log-time">[${time}]</span> ${message}`;

    container.appendChild(entry);
    container.scrollTop = container.scrollHeight;

    // 限制日志条数
    while (container.children.length > 100) {
        container.removeChild(container.firstChild);
    }
}

function clearLog() {
    const container = document.getElementById('log-container');
    if (container) {
        container.innerHTML = '';
    }
}

// ==================== 通知 ====================

function showNotification(message, type = 'info') {
    // Toast 通知（可见）
    let container = document.getElementById('toast-container');
    if (!container) {
        container = document.createElement('div');
        container.id = 'toast-container';
        container.className = 'toast-container';
        document.body.appendChild(container);
    }

    const t = String(type || 'info').toLowerCase();
    const toast = document.createElement('div');
    toast.className = `toast toast-${t}`;

    const titleMap = {
        success: '成功',
        warning: '提示',
        error: '错误',
        info: '信息'
    };
    const title = titleMap[t] || '信息';

    toast.innerHTML = `
        <div class="toast-title">${title}</div>
        <div class="toast-msg">${escapeHtml(String(message || ''))}</div>
    `;
    container.appendChild(toast);

    setTimeout(() => {
        toast.remove();
    }, 2600);
}

function escapeHtml(str) {
    return str
        .replace(/&/g, '&amp;')
        .replace(/</g, '&lt;')
        .replace(/>/g, '&gt;')
        .replace(/"/g, '&quot;')
        .replace(/'/g, '&#039;');
}

// ==================== 初始化 ====================

document.addEventListener('DOMContentLoaded', () => {
    console.log('[App] 页面加载完成');

    // 绑定Tab切换
    document.querySelectorAll('.tab-btn[data-tab]').forEach(btn => {
        btn.addEventListener('click', () => switchTab(btn.dataset.tab));
    });

    // 绑定目标选择
    document.querySelectorAll('.target-option').forEach(opt => {
        opt.addEventListener('click', () => setTarget(opt.dataset.target));
    });

    // 绑定关节滑杆事件
    bindJointSliderEvents();

    // 绑定笛卡尔滑杆事件
    bindCartesianSliderEvents();
    bindJogHoldEvents();
    setMotionMode('CARTESIAN_PTP');

    // 默认显示第一个Tab
    switchTab('manual');

    addLog('系统初始化完成', 'success');
    updateMonitor(); // Render empty rows initially
    // 在未连接状态下，优先从本地零点文件加载一次显示
    loadZeroOffsetFromFile();
    // 先把速度滑杆与输入框同步一次（连接后会再次初始化）
    initSpeedSlider();
});

// ==================== 导出全局函数 ====================

window.connectArms = connectArms;
window.disconnectArms = disconnectArms;
window.initArms = initArms;
window.deactivateArms = deactivateArms;
window.disableArms = disableArms;
window.emergencyStop = emergencyStop;
window.goToZero = goToZero;
window.setTarget = setTarget;
window.playTrajectory = playTrajectory;
window.pauseTrajectory = pauseTrajectory;
window.resumeTrajectory = resumeTrajectory;
window.stopTrajectory = stopTrajectory;
window.playPreset = playPreset;
window.setSpeed = setSpeed;
window.setCustomSpeed = setCustomSpeed;
window.enableFreedrive = enableFreedrive;
window.disableFreedrive = disableFreedrive;
window.jogMotor = jogMotor;
window.zeroMotor = zeroMotor;
window.calibrateZero = calibrateZero;
window.loadZeroOffset = loadZeroOffset;
window.recordPoint = recordPoint;
window.switchTab = switchTab;
window.clearLog = clearLog;
window.cartesianJog = cartesianJog;
window.cartesianMoveTo = cartesianMoveTo;
window.cartesianSliderMove = cartesianSliderMove;
window.cartesianInputMove = cartesianInputMove;
window.trajectoryNewPrompt = trajectoryNewPrompt;
window.trajectoryListAndLoad = trajectoryListAndLoad;
window.trajectoryListAndDelete = trajectoryListAndDelete;
window.trajectoryRefreshList = trajectoryRefreshList;
window.trajectoryAddPoint = trajectoryAddPoint;
window.trajectoryRemovePoint = trajectoryRemovePoint;
window.trajectoryMovePoint = trajectoryMovePoint;
window.trajectorySave = trajectorySave;
window.trajectoryPreview = trajectoryPreview;
window.trajectoryCloseEdit = trajectoryCloseEdit;
window.trajFileAction = trajFileAction;
window.setMotionMode = setMotionMode;
window.executeMotion = executeMotion;
window.pauseMotion = pauseMotion;
window.resumeMotion = resumeMotion;
window.cancelMotion = cancelMotion;
window.resetMotionConstraints = resetMotionConstraints;
