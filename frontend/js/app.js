/**
 * Digital Twin of a Segway / Inverted Pendulum
 * Frontend Canvas Physics Renderer & Telemetry Controller
 */

const API_BASE_URL = 'https://inverted-pendulum-gzf5.onrender.com';

// Global Application State
const appState = {
    isRunning: true,
    simulationSpeed: 1.0,
    controlMode: 'state_feedback', // 'state_feedback' or 'observer_feedback'
    noiseEnabled: false,
    noiseStd: 0.003,
    disturbancePending: 0.0,
    lastFrameTime: performance.now(),
    stepIntervalMs: 50, // 20 Hz backend step rate (dt = 0.05s)
    lastStepTime: 0,
    apiOnline: false,
    wheelRotation: 0.0,
    
    // Telemetry cache
    currentTelemetry: {
        time: 0.0,
        state: { p: 0.0, p_dot: 0.0, theta: 0.15, theta_dot: 0.0, theta_deg: 8.59 },
        estimated_state: { p: 0.0, p_dot: 0.0, theta: 0.0, theta_dot: 0.0, theta_deg: 0.0 },
        error: [0.0, 0.0, 0.15, 0.0],
        error_norm: 0.15,
        control_input_u: 0.0,
        disturbance: 0.0,
        measurement_y: 0.0,
        noise_enabled: false,
        noise_std: 0.003,
        stable: true
    }
};

// DOM Elements
let canvas, ctx;
let btnReset, btnImpulseLeft, btnImpulseRight, btnToggleNoise, btnPlayPause;
let badgeStability, badgeConnection, textTime;
let metricP, metricPDot, metricTheta, metricThetaDot, metricU, metricErrNorm;

/**
 * Initialize Canvas and UI Event Handlers on DOM Load
 */
document.addEventListener('DOMContentLoaded', () => {
    canvas = document.getElementById('physicsCanvas');
    ctx = canvas.getContext('2d');

    // Setup High DPI Canvas
    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    // Cache UI elements
    badgeStability = document.getElementById('badgeStability');
    badgeConnection = document.getElementById('badgeConnection');
    textTime = document.getElementById('textTime');

    metricP = document.getElementById('metricP');
    metricPDot = document.getElementById('metricPDot');
    metricTheta = document.getElementById('metricTheta');
    metricThetaDot = document.getElementById('metricThetaDot');
    metricU = document.getElementById('metricU');
    metricErrNorm = document.getElementById('metricErrNorm');

    btnReset = document.getElementById('btnReset');
    btnImpulseLeft = document.getElementById('btnImpulseLeft');
    btnImpulseRight = document.getElementById('btnImpulseRight');
    btnToggleNoise = document.getElementById('btnToggleNoise');
    btnPlayPause = document.getElementById('btnPlayPause');

    // Attach Event Listeners
    btnReset.addEventListener('click', () => handleReset());
    btnImpulseLeft.addEventListener('click', () => applyDisturbance(-15.0));
    btnImpulseRight.addEventListener('click', () => applyDisturbance(15.0));
    btnToggleNoise.addEventListener('click', () => toggleSensorNoise());
    btnPlayPause.addEventListener('click', () => togglePlayPause());

    // Control Mode Switcher
    const radioState = document.getElementById('modeStateFeedback');
    const radioObs = document.getElementById('modeObserverFeedback');
    if (radioState && radioObs) {
        radioState.addEventListener('change', () => setControlMode('state_feedback'));
        radioObs.addEventListener('change', () => setControlMode('observer_feedback'));
    }

    // Keyboard Shortcuts
    window.addEventListener('keydown', (e) => {
        if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
        if (e.code === 'Space') {
            e.preventDefault();
            togglePlayPause();
        } else if (e.code === 'KeyR') {
            e.preventDefault();
            handleReset();
        } else if (e.code === 'ArrowLeft') {
            e.preventDefault();
            applyDisturbance(-15.0);
        } else if (e.code === 'ArrowRight') {
            e.preventDefault();
            applyDisturbance(15.0);
        } else if (e.code === 'KeyN') {
            e.preventDefault();
            toggleSensorNoise();
        }
    });

    // Initialize telemetry graphs
    initCharts();

    // Fetch initial model metadata and initial status
    fetchModelInfo();
    fetchStatus();

    // Start 60 FPS Request Animation Frame Loop
    requestAnimationFrame(animationLoop);
});

/**
 * Handle Canvas Retina / High DPI Resizing
 */
function resizeCanvas() {
    const rect = canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    canvas.width = rect.width * dpr;
    canvas.height = rect.height * dpr;
    ctx.resetTransform();
    ctx.scale(dpr, dpr);
}

/**
 * Main 60 FPS requestAnimationFrame rendering and step scheduling loop
 */
function animationLoop(timestamp) {
    const elapsedSinceStep = timestamp - appState.lastStepTime;

    // Advance simulation step if running and interval elapsed
    if (appState.isRunning && elapsedSinceStep >= (appState.stepIntervalMs / appState.simulationSpeed)) {
        stepSimulation();
        appState.lastStepTime = timestamp;
    }

    // Render Canvas scene at full display refresh rate
    renderPhysicsScene();

    requestAnimationFrame(animationLoop);
}

/**
 * Send POST /api/step request to FastAPI backend
 */
async function stepSimulation() {
    const payload = {
        force_disturbance: appState.disturbancePending,
        noise_enabled: appState.noiseEnabled,
        noise_std: appState.noiseStd
    };

    // Reset pending disturbance once dispatched
    appState.disturbancePending = 0.0;

    try {
        const response = await fetch(`${API_BASE_URL}/api/step`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(payload)
        });

        if (!response.ok) throw new Error(`HTTP error ${response.status}`);

        const telemetry = await response.json();
        appState.currentTelemetry = telemetry;
        setConnectionStatus(true);
        updateUI(telemetry);
        updateCharts(telemetry);
    } catch (err) {
        setConnectionStatus(false);
    }
}

/**
 * Send POST /api/reset request
 */
async function handleReset(theta0 = 0.15) {
    try {
        const response = await fetch(`${API_BASE_URL}/api/reset`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ theta_0: theta0, p_0: 0.0, reset_observer: true })
        });
        if (response.ok) {
            const data = await response.json();
            appState.currentTelemetry = data;
            resetCharts();
            updateUI(data);
            updateCharts(data);
        }
    } catch (err) {
        setConnectionStatus(false);
    }
}

/**
 * Fetch initial system status
 */
async function fetchStatus() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/status`);
        if (response.ok) {
            const data = await response.json();
            appState.currentTelemetry = data;
            setConnectionStatus(true);
            updateUI(data);
            updateCharts(data);
        }
    } catch (err) {
        setConnectionStatus(false);
    }
}

/**
 * Fetch model information (matrices, gains, poles)
 */
async function fetchModelInfo() {
    try {
        const response = await fetch(`${API_BASE_URL}/api/model_info`);
        if (response.ok) {
            const info = await response.json();
            renderModelInfoModal(info);
        }
    } catch (err) {
        // Silent catch for background info
    }
}

/**
 * Configure control mode
 */
async function setControlMode(mode) {
    appState.controlMode = mode;
    try {
        await fetch(`${API_BASE_URL}/api/configure`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ control_mode: mode })
        });
    } catch (err) {}
}

/**
 * Toggle sensor measurement noise
 */
async function toggleSensorNoise() {
    appState.noiseEnabled = !appState.noiseEnabled;
    const btn = document.getElementById('btnToggleNoise');
    const badge = document.getElementById('noiseStatusBadge');
    
    if (appState.noiseEnabled) {
        btn.classList.add('border-amber-500', 'text-amber-400');
        badge.textContent = 'ON (σ = 3mm)';
        badge.className = 'px-2 py-0.5 rounded text-[10px] font-mono bg-amber-500/20 text-amber-300 border border-amber-500/40';
    } else {
        btn.classList.remove('border-amber-500', 'text-amber-400');
        badge.textContent = 'OFF';
        badge.className = 'px-2 py-0.5 rounded text-[10px] font-mono bg-slate-800 text-slate-400 border border-slate-700';
    }

    try {
        await fetch(`${API_BASE_URL}/api/configure`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ noise_enabled: appState.noiseEnabled })
        });
    } catch (err) {}
}

/**
 * Trigger disturbance kick
 */
function applyDisturbance(magnitude) {
    appState.disturbancePending += magnitude;
    // Brief visual flash on button
    const flashEl = magnitude > 0 ? btnImpulseRight : btnImpulseLeft;
    flashEl.classList.add('ring-2', 'ring-red-400', 'bg-red-950/40');
    setTimeout(() => {
        flashEl.classList.remove('ring-2', 'ring-red-400', 'bg-red-950/40');
    }, 200);
}

/**
 * Play/Pause simulation loop
 */
function togglePlayPause() {
    appState.isRunning = !appState.isRunning;
    const icon = document.getElementById('playPauseIcon');
    const text = document.getElementById('playPauseText');
    if (appState.isRunning) {
        icon.innerHTML = `<path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M10 9v6m4-6v6m7-3a9 9 0 11-18 0 9 9 0 0118 0z" />`;
        text.textContent = 'Pause';
        btnPlayPause.className = 'px-3 py-1.5 rounded-lg text-xs font-medium bg-slate-800 hover:bg-slate-700 text-slate-300 border border-slate-700 flex items-center gap-1.5 transition';
    } else {
        icon.innerHTML = `<path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z" /><path stroke-linecap="round" stroke-linejoin="round" stroke-width="2" d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z" />`;
        text.textContent = 'Resume';
        btnPlayPause.className = 'px-3 py-1.5 rounded-lg text-xs font-medium bg-emerald-600 hover:bg-emerald-500 text-white flex items-center gap-1.5 transition shadow-lg shadow-emerald-900/30';
    }
}

/**
 * Update UI telemetry widgets
 */
function updateUI(data) {
    if (!data) return;

    textTime.textContent = `${Number(data.time).toFixed(2)}s`;

    // Metrics Cards
    metricP.textContent = `${Number(data.state.p).toFixed(3)} m`;
    metricPDot.textContent = `${Number(data.state.p_dot).toFixed(3)} m/s`;
    metricTheta.textContent = `${Number(data.state.theta).toFixed(3)} rad (${Number(data.state.theta_deg).toFixed(1)}°)`;
    metricThetaDot.textContent = `${Number(data.state.theta_dot).toFixed(3)} rad/s`;
    metricU.textContent = `${Number(data.control_input_u).toFixed(2)} N`;
    metricErrNorm.textContent = `${Number(data.error_norm).toFixed(4)}`;

    // Stability Badge
    const thetaAbs = Math.abs(data.state.theta);
    if (thetaAbs < 0.05) {
        badgeStability.textContent = 'STABLE (EQUILIBRIUM)';
        badgeStability.className = 'px-2.5 py-1 rounded-full text-xs font-semibold bg-emerald-500/20 text-emerald-400 border border-emerald-500/40 flex items-center gap-1.5';
    } else if (thetaAbs < 0.35) {
        badgeStability.textContent = 'CORRECTING (TRANSIENT)';
        badgeStability.className = 'px-2.5 py-1 rounded-full text-xs font-semibold bg-amber-500/20 text-amber-400 border border-amber-500/40 flex items-center gap-1.5 animate-pulse';
    } else {
        badgeStability.textContent = 'UNSTABLE / FALLING';
        badgeStability.className = 'px-2.5 py-1 rounded-full text-xs font-semibold bg-rose-500/20 text-rose-400 border border-rose-500/40 flex items-center gap-1.5';
    }
}

/**
 * Connection status pill
 */
function setConnectionStatus(online) {
    appState.apiOnline = online;
    if (online) {
        badgeConnection.innerHTML = `<span class="w-2 h-2 rounded-full bg-emerald-400 animate-ping"></span><span class="w-2 h-2 rounded-full bg-emerald-400 absolute"></span><span class="ml-3 text-emerald-400 font-mono text-xs">API Connected</span>`;
    } else {
        badgeConnection.innerHTML = `<span class="w-2 h-2 rounded-full bg-rose-500"></span><span class="ml-1 text-rose-400 font-mono text-xs">Offline (Connecting...)</span>`;
    }
}

/**
 * Render physical Segway / Inverted Pendulum scene on HTML5 Canvas
 */
function renderPhysicsScene() {
    const rect = canvas.getBoundingClientRect();
    const width = rect.width;
    const height = rect.height;

    ctx.clearRect(0, 0, width, height);

    // Coordinate System Setup
    // Track center is at horizontal center, 72% down the canvas
    const originX = width / 2;
    const groundY = height * 0.72;
    const pixelsPerMeter = Math.min(width * 0.28, 200); // Scale factor (1m = ~200px)

    // Current Physical State
    const p = appState.currentTelemetry.state.p;
    const theta = appState.currentTelemetry.state.theta;
    const theta_hat = appState.currentTelemetry.estimated_state.theta;
    const p_hat = appState.currentTelemetry.estimated_state.p;
    const u = appState.currentTelemetry.control_input_u;
    const dist = appState.currentTelemetry.disturbance;

    // Cart Screen Position
    const cartX = originX + p * pixelsPerMeter;
    const cartY = groundY - 24; // Cart center height above track
    const cartW = 86;
    const cartH = 34;
    const wheelRadius = 15;
    const rodLengthPx = 0.5 * pixelsPerMeter * 1.5; // Scaled rod length (~150px)

    // 1. Draw Background Engineering Grid
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.035)';
    ctx.lineWidth = 1;
    const gridSize = 30;
    for (let x = 0; x < width; x += gridSize) {
        ctx.beginPath();
        ctx.moveTo(x, 0);
        ctx.lineTo(x, height);
        ctx.stroke();
    }
    for (let y = 0; y < height; y += gridSize) {
        ctx.beginPath();
        ctx.moveTo(0, y);
        ctx.lineTo(width, y);
        ctx.stroke();
    }

    // 2. Draw Horizontal Guideline Track & Metric Scale
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(20, groundY);
    ctx.lineTo(width - 20, groundY);
    ctx.stroke();

    // Metric Ticks & Labels along track
    const maxMeters = Math.floor((width / 2) / pixelsPerMeter);
    ctx.fillStyle = '#64748b';
    ctx.font = '10px monospace';
    ctx.textAlign = 'center';
    for (let m = -maxMeters; m <= maxMeters; m += 0.5) {
        const tx = originX + m * pixelsPerMeter;
        if (tx < 30 || tx > width - 30) continue;
        const isMajor = Math.abs(m % 1.0) < 0.01;
        ctx.strokeStyle = isMajor ? '#475569' : '#334155';
        ctx.lineWidth = isMajor ? 1.5 : 1;
        ctx.beginPath();
        ctx.moveTo(tx, groundY - (isMajor ? 8 : 4));
        ctx.lineTo(tx, groundY + (isMajor ? 8 : 4));
        ctx.stroke();
        if (isMajor) {
            ctx.fillText(`${m > 0 ? '+' : ''}${m.toFixed(1)}m`, tx, groundY + 20);
        }
    }

    // Origin Zero Marker
    ctx.strokeStyle = 'rgba(56, 189, 248, 0.4)';
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.moveTo(originX, groundY - 180);
    ctx.lineTo(originX, groundY + 25);
    ctx.stroke();
    ctx.setLineDash([]);

    // 3. Draw Observer Ghost Estimate (θ̂ and p̂)
    if (Math.abs(p - p_hat) > 0.001 || Math.abs(theta - theta_hat) > 0.005) {
        const obsCartX = originX + p_hat * pixelsPerMeter;
        const obsPivotX = obsCartX;
        const obsPivotY = cartY - cartH / 2;
        const obsTipX = obsPivotX + rodLengthPx * Math.sin(theta_hat);
        const obsTipY = obsPivotY - rodLengthPx * Math.cos(theta_hat);

        // Dashed Amber Line representing Observer State
        ctx.save();
        ctx.strokeStyle = 'rgba(245, 158, 11, 0.65)';
        ctx.setLineDash([6, 5]);
        ctx.lineWidth = 2.5;
        ctx.beginPath();
        ctx.moveTo(obsPivotX, obsPivotY);
        ctx.lineTo(obsTipX, obsTipY);
        ctx.stroke();

        // Observer COM Bob
        ctx.fillStyle = 'rgba(245, 158, 11, 0.7)';
        ctx.beginPath();
        ctx.arc(obsTipX, obsTipY, 8, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#f59e0b';
        ctx.font = '10px monospace';
        ctx.fillText('Observer θ̂', obsTipX + 12, obsTipY + 4);
        ctx.restore();
    }

    // 4. Draw Physical Cart Body (Segway Platform)
    // Cart Shadow
    ctx.fillStyle = 'rgba(0, 0, 0, 0.35)';
    ctx.beginPath();
    ctx.ellipse(cartX, groundY + 2, cartW * 0.55, 6, 0, 0, Math.PI * 2);
    ctx.fill();

    // Cart Wheels
    const leftWheelX = cartX - cartW / 3;
    const rightWheelX = cartX + cartW / 3;
    const wheelY = groundY - wheelRadius;

    // Update wheel rotation angle based on velocity
    appState.wheelRotation += (appState.currentTelemetry.state.p_dot * 0.1);

    [leftWheelX, rightWheelX].forEach(wx => {
        // Outer Tire
        ctx.fillStyle = '#0f172a';
        ctx.strokeStyle = '#38bdf8';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(wx, wheelY, wheelRadius, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Wheel Hub and Spokes
        ctx.strokeStyle = '#64748b';
        ctx.lineWidth = 1.5;
        for (let a = 0; a < 3; a++) {
            const spokeAng = appState.wheelRotation + (a * (Math.PI * 2 / 3));
            ctx.beginPath();
            ctx.moveTo(wx, wheelY);
            ctx.lineTo(wx + (wheelRadius - 3) * Math.cos(spokeAng), wheelY + (wheelRadius - 3) * Math.sin(spokeAng));
            ctx.stroke();
        }

        ctx.fillStyle = '#38bdf8';
        ctx.beginPath();
        ctx.arc(wx, wheelY, 3, 0, Math.PI * 2);
        ctx.fill();
    });

    // Cart Chassis (Rounded Rect)
    const chassisGradient = ctx.createLinearGradient(cartX - cartW / 2, cartY - cartH / 2, cartX + cartW / 2, cartY + cartH / 2);
    chassisGradient.addColorStop(0, '#1e293b');
    chassisGradient.addColorStop(0.5, '#334155');
    chassisGradient.addColorStop(1, '#0f172a');

    ctx.fillStyle = chassisGradient;
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 2;
    drawRoundedRect(ctx, cartX - cartW / 2, cartY - cartH / 2, cartW, cartH, 8);
    ctx.fill();
    ctx.stroke();

    // Chassis Glow Accent
    ctx.fillStyle = '#38bdf8';
    ctx.fillRect(cartX - cartW / 3, cartY - 3, (cartW * 2) / 3, 3);

    // 5. Draw Pivot and Rigid Pendulum Pole
    const pivotX = cartX;
    const pivotY = cartY - cartH / 2;
    const tipX = pivotX + rodLengthPx * Math.sin(theta);
    const tipY = pivotY - rodLengthPx * Math.cos(theta);

    // Angle Arc indicator
    ctx.strokeStyle = 'rgba(52, 211, 153, 0.45)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    const arcRadius = 38;
    const startAngle = -Math.PI / 2;
    const endAngle = -Math.PI / 2 + theta;
    ctx.arc(pivotX, pivotY, arcRadius, Math.min(startAngle, endAngle), Math.max(startAngle, endAngle));
    ctx.stroke();

    // Rigid Pole Rod
    const rodGradient = ctx.createLinearGradient(pivotX, pivotY, tipX, tipY);
    rodGradient.addColorStop(0, '#0284c7');
    rodGradient.addColorStop(0.5, '#38bdf8');
    rodGradient.addColorStop(1, '#67e8f9');

    ctx.strokeStyle = rodGradient;
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.moveTo(pivotX, pivotY);
    ctx.lineTo(tipX, tipY);
    ctx.stroke();

    // Pendulum Center of Mass Bob (m = 0.1 kg)
    const bobRadius = 14;
    const bobGradient = ctx.createRadialGradient(tipX - 3, tipY - 3, 2, tipX, tipY, bobRadius);
    bobGradient.addColorStop(0, '#6ee7b7');
    bobGradient.addColorStop(0.6, '#10b981');
    bobGradient.addColorStop(1, '#064e3b');

    ctx.fillStyle = bobGradient;
    ctx.strokeStyle = '#34d399';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(tipX, tipY, bobRadius, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Pivot Bearing Joint
    ctx.fillStyle = '#e2e8f0';
    ctx.strokeStyle = '#0f172a';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.arc(pivotX, pivotY, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // 6. Draw Control Force Vector u (Purple/Cyan Arrow)
    if (Math.abs(u) > 0.1) {
        const arrowLen = Math.min(Math.abs(u) * 2.2, 90);
        const arrowDir = u > 0 ? 1 : -1;
        const arrowStartX = cartX;
        const arrowStartY = cartY;
        const arrowEndX = arrowStartX + arrowDir * arrowLen;

        ctx.strokeStyle = '#a855f7';
        ctx.fillStyle = '#a855f7';
        ctx.lineWidth = 3;
        drawArrow(ctx, arrowStartX, arrowStartY, arrowEndX, arrowStartY, 8);
        ctx.font = '10px monospace';
        ctx.textAlign = arrowDir > 0 ? 'left' : 'right';
        ctx.fillText(`u: ${u > 0 ? '+' : ''}${u.toFixed(1)} N`, arrowEndX + arrowDir * 6, arrowStartY - 6);
    }

    // 7. Draw Disturbance Force Vector (Orange/Red Flash Arrow)
    if (Math.abs(dist) > 0.1) {
        const distLen = Math.min(Math.abs(dist) * 2.5, 80);
        const distDir = dist > 0 ? 1 : -1;
        const distStartX = tipX;
        const distStartY = tipY;
        const distEndX = distStartX + distDir * distLen;

        ctx.strokeStyle = '#f97316';
        ctx.fillStyle = '#f97316';
        ctx.lineWidth = 3.5;
        drawArrow(ctx, distStartX, distStartY, distEndX, distStartY, 9);
        ctx.font = '11px monospace';
        ctx.fillText(`F_dist: ${dist.toFixed(1)} N`, distEndX + distDir * 8, distStartY - 8);
    }

    // 8. Dynamic Status Overlay Text
    ctx.fillStyle = '#94a3b8';
    ctx.font = '11px monospace';
    ctx.textAlign = 'left';
    ctx.fillText(`Cart Position p: ${p.toFixed(3)} m`, 16, 26);
    ctx.fillText(`Tilt Angle θ: ${theta.toFixed(3)} rad (${(theta * 180 / Math.PI).toFixed(1)}°)`, 16, 42);
    ctx.fillText(`Control Force u: ${u.toFixed(2)} N`, 16, 58);
}

/**
 * Utility to draw rounded rectangle on canvas
 */
function drawRoundedRect(ctx, x, y, width, height, radius) {
    ctx.beginPath();
    ctx.moveTo(x + radius, y);
    ctx.lineTo(x + width - radius, y);
    ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
    ctx.lineTo(x + width, y + height - radius);
    ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
    ctx.lineTo(x + radius, y + height);
    ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
    ctx.lineTo(x, y + radius);
    ctx.quadraticCurveTo(x, y, x + radius, y);
    ctx.closePath();
}

/**
 * Utility to draw vector arrow
 */
function drawArrow(ctx, fromX, fromY, toX, toY, headLength = 7) {
    const dx = toX - fromX;
    const dy = toY - fromY;
    const angle = Math.atan2(dy, dx);
    ctx.beginPath();
    ctx.moveTo(fromX, fromY);
    ctx.lineTo(toX, toY);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(toX, toY);
    ctx.lineTo(toX - headLength * Math.cos(angle - Math.PI / 6), toY - headLength * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(toX - headLength * Math.cos(angle + Math.PI / 6), toY - headLength * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.fill();
}

/**
 * Populate Model Metadata Modal/Card
 */
function renderModelInfoModal(info) {
    const container = document.getElementById('modelInfoContent');
    if (!container || !info) return;

    const A = info.matrices.A;
    const B = info.matrices.B;
    const K = info.gains.K[0];
    const L = info.gains.L;

    container.innerHTML = `
        <div class="grid grid-cols-1 md:grid-cols-2 gap-4 text-xs font-mono text-slate-300">
            <div class="p-3 bg-slate-900/60 rounded border border-slate-800">
                <div class="text-sky-400 font-bold mb-1">State Matrix A (4x4)</div>
                <pre class="overflow-x-auto">${A.map(row => row.map(v => v.toFixed(3).padStart(7)).join(' ')).join('\n')}</pre>
            </div>
            <div class="p-3 bg-slate-900/60 rounded border border-slate-800">
                <div class="text-indigo-400 font-bold mb-1">Input Matrix B (4x1)</div>
                <pre>${B.map(row => row[0].toFixed(3).padStart(7)).join('\n')}</pre>
            </div>
            <div class="p-3 bg-slate-900/60 rounded border border-slate-800">
                <div class="text-emerald-400 font-bold mb-1">State Feedback Gain K [u = -Kx]</div>
                <pre class="overflow-x-auto">[${K.map(v => v.toFixed(3)).join(', ')}]</pre>
                <div class="text-[10px] text-slate-500 mt-1">Poles placed at: [-2.0, -2.2, -1.8, -2.5]</div>
            </div>
            <div class="p-3 bg-slate-900/60 rounded border border-slate-800">
                <div class="text-amber-400 font-bold mb-1">Observer Gain L [4x1]</div>
                <pre class="overflow-x-auto">${L.map((v, i) => `L[${i}]: ${v[0].toFixed(2).padStart(9)}`).join('\n')}</pre>
                <div class="text-[10px] text-slate-500 mt-1">Observer poles placed at: [-5.0, -5.5, -6.0, -6.5]</div>
            </div>
        </div>
    `;
}
