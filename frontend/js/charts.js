/**
 * Digital Twin of a Segway / Inverted Pendulum
 * Real-time Chart.js Telemetry and Luenberger Observer Tracking
 */

const MAX_CHART_POINTS = 80;

let stateChart = null;
let observerChart = null;
let controlChart = null;

// Rolling buffers
const timeLabels = [];
const bufferP = [];
const bufferPDot = [];
const bufferTheta = [];
const bufferThetaDot = [];

const bufferThetaTrue = [];
const bufferThetaEst = [];
const bufferObsError = [];

const bufferControlU = [];
const bufferDisturbance = [];

/**
 * Initialize all Chart.js instances with dark engineering theme.
 */
function initCharts() {
    const commonOptions = {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        interaction: {
            mode: 'index',
            intersect: false,
        },
        plugins: {
            legend: {
                labels: {
                    color: '#cbd5e1',
                    font: { family: 'ui-monospace, SFMono-Regular, Menlo, monospace', size: 11 },
                    boxWidth: 12,
                    boxHeight: 12,
                    usePointStyle: true
                }
            },
            tooltip: {
                backgroundColor: '#0f172a',
                titleColor: '#f8fafc',
                bodyColor: '#cbd5e1',
                borderColor: '#334155',
                borderWidth: 1,
                padding: 8,
                titleFont: { size: 11, weight: 'bold' },
                bodyFont: { size: 11 }
            }
        },
        scales: {
            x: {
                grid: { color: 'rgba(148, 163, 184, 0.08)' },
                ticks: {
                    color: '#64748b',
                    maxTicksLimit: 8,
                    font: { size: 10, family: 'monospace' }
                },
                title: {
                    display: true,
                    text: 'Simulation Time (s)',
                    color: '#64748b',
                    font: { size: 10 }
                }
            },
            y: {
                grid: { color: 'rgba(148, 163, 184, 0.08)' },
                ticks: {
                    color: '#94a3b8',
                    font: { size: 10, family: 'monospace' }
                }
            }
        }
    };

    // 1. Chart 1: State Trajectories [p, p_dot, theta, theta_dot]
    const ctxState = document.getElementById('stateChart').getContext('2d');
    stateChart = new Chart(ctxState, {
        type: 'line',
        data: {
            labels: timeLabels,
            datasets: [
                {
                    label: 'p (Position, m)',
                    data: bufferP,
                    borderColor: '#38bdf8',
                    backgroundColor: '#38bdf8',
                    borderWidth: 1.8,
                    pointRadius: 0,
                    tension: 0.15
                },
                {
                    label: 'p_dot (Vel, m/s)',
                    data: bufferPDot,
                    borderColor: '#818cf8',
                    backgroundColor: '#818cf8',
                    borderWidth: 1.5,
                    borderDash: [3, 2],
                    pointRadius: 0,
                    tension: 0.15
                },
                {
                    label: 'θ (Angle, rad)',
                    data: bufferTheta,
                    borderColor: '#34d399',
                    backgroundColor: '#34d399',
                    borderWidth: 2.2,
                    pointRadius: 0,
                    tension: 0.15
                },
                {
                    label: 'θ_dot (AngVel, rad/s)',
                    data: bufferThetaDot,
                    borderColor: '#f472b6',
                    backgroundColor: '#f472b6',
                    borderWidth: 1.5,
                    borderDash: [3, 2],
                    pointRadius: 0,
                    tension: 0.15
                }
            ]
        },
        options: {
            ...commonOptions,
            plugins: {
                ...commonOptions.plugins,
                title: {
                    display: true,
                    text: 'CHART 1: STATE TRAJECTORIES [p, ṗ, θ, θ̇]',
                    color: '#94a3b8',
                    font: { size: 11, weight: '600' },
                    align: 'start',
                    padding: { bottom: 8 }
                }
            }
        }
    });

    // 2. Chart 2: True vs. Estimated Pendulum Angle θ(t) vs θ_hat(t)
    const ctxObserver = document.getElementById('observerChart').getContext('2d');
    observerChart = new Chart(ctxObserver, {
        type: 'line',
        data: {
            labels: timeLabels,
            datasets: [
                {
                    label: 'θ True (Plant Angle, rad)',
                    data: bufferThetaTrue,
                    borderColor: '#10b981',
                    backgroundColor: '#10b981',
                    borderWidth: 2.2,
                    pointRadius: 0,
                    tension: 0.15
                },
                {
                    label: 'θ̂ Estimated (Observer, rad)',
                    data: bufferThetaEst,
                    borderColor: '#f59e0b',
                    backgroundColor: '#f59e0b',
                    borderWidth: 2.0,
                    borderDash: [5, 4],
                    pointRadius: 0,
                    tension: 0.15
                },
                {
                    label: 'Angle Error |θ - θ̂| (rad)',
                    data: bufferObsError,
                    borderColor: '#ef4444',
                    backgroundColor: 'rgba(239, 68, 68, 0.12)',
                    fill: true,
                    borderWidth: 1.2,
                    pointRadius: 0,
                    tension: 0.15
                }
            ]
        },
        options: {
            ...commonOptions,
            plugins: {
                ...commonOptions.plugins,
                title: {
                    display: true,
                    text: 'CHART 2: OBSERVER TRACKING (TRUE θ vs ESTIMATED θ̂)',
                    color: '#94a3b8',
                    font: { size: 11, weight: '600' },
                    align: 'start',
                    padding: { bottom: 8 }
                }
            }
        }
    });

    // 3. Chart 3: Control Input Effort u(t) & Disturbance
    const ctxControl = document.getElementById('controlChart').getContext('2d');
    controlChart = new Chart(ctxControl, {
        type: 'line',
        data: {
            labels: timeLabels,
            datasets: [
                {
                    label: 'Control Force u (N)',
                    data: bufferControlU,
                    borderColor: '#a855f7',
                    backgroundColor: 'rgba(168, 85, 247, 0.1)',
                    fill: true,
                    borderWidth: 1.8,
                    pointRadius: 0,
                    tension: 0.15
                },
                {
                    label: 'Disturbance Force (N)',
                    data: bufferDisturbance,
                    borderColor: '#f97316',
                    backgroundColor: '#f97316',
                    borderWidth: 1.5,
                    borderDash: [2, 2],
                    pointRadius: 0,
                    tension: 0.15
                }
            ]
        },
        options: {
            ...commonOptions,
            plugins: {
                ...commonOptions.plugins,
                title: {
                    display: true,
                    text: 'CHART 3: CONTROL EFFORT u = -Kx & DISTURBANCE IMPULSE',
                    color: '#94a3b8',
                    font: { size: 11, weight: '600' },
                    align: 'start',
                    padding: { bottom: 8 }
                }
            }
        }
    });
}

/**
 * Append incoming telemetry data packet to charts and slide the window.
 * @param {Object} data - Packet from /api/step or /api/status
 */
function updateCharts(data) {
    if (!stateChart || !observerChart || !controlChart) return;

    const t = Number(data.time).toFixed(2);
    timeLabels.push(t);

    bufferP.push(data.state.p);
    bufferPDot.push(data.state.p_dot);
    bufferTheta.push(data.state.theta);
    bufferThetaDot.push(data.state.theta_dot);

    bufferThetaTrue.push(data.state.theta);
    bufferThetaEst.push(data.estimated_state.theta);
    bufferObsError.push(Math.abs(data.state.theta - data.estimated_state.theta));

    bufferControlU.push(data.control_input_u);
    bufferDisturbance.push(data.disturbance || 0.0);

    // Maintain fixed window
    if (timeLabels.length > MAX_CHART_POINTS) {
        timeLabels.shift();
        bufferP.shift();
        bufferPDot.shift();
        bufferTheta.shift();
        bufferThetaDot.shift();

        bufferThetaTrue.shift();
        bufferThetaEst.shift();
        bufferObsError.shift();

        bufferControlU.shift();
        bufferDisturbance.shift();
    }

    stateChart.update('none');
    observerChart.update('none');
    controlChart.update('none');
}

/**
 * Clear all chart buffers on simulation reset.
 */
function resetCharts() {
    timeLabels.length = 0;
    bufferP.length = 0;
    bufferPDot.length = 0;
    bufferTheta.length = 0;
    bufferThetaDot.length = 0;

    bufferThetaTrue.length = 0;
    bufferThetaEst.length = 0;
    bufferObsError.length = 0;

    bufferControlU.length = 0;
    bufferDisturbance.length = 0;

    if (stateChart) stateChart.update();
    if (observerChart) observerChart.update();
    if (controlChart) controlChart.update();
}
