# Digital Twin of a Segway / Inverted Pendulum

A production-grade, interactive web application modeling an inverted pendulum on a cart (Segway digital twin). The application combines a Python **FastAPI** backend computing continuous-time linear and nonlinear control systems math with an animated, dark-themed frontend built with **Tailwind CSS**, **HTML5 Canvas**, and **Chart.js**.

---

## 1. Syllabus & Control Engineering Foundations

This project directly implements the core curriculum of **Control Systems II / Advanced State-Space Control Theory**:
1. **Lagrangian Formulation of Nonlinear Systems:** Deriving coupled equations of motion for multi-degree-of-freedom robotic systems.
2. **Jacobian Linearization:** Linearizing nonlinear dynamics around an unstable upright operating point ($\theta = 0$).
3. **Controllability & Observability:** Validating that the rank of the controllability matrix $\mathcal{C}$ and observability matrix $\mathcal{O}$ equals the state dimension ($n = 4$).
4. **State-Feedback Pole Placement:** Designing gain matrix $K$ via Bass-Gura / Ackermann's formula / `control.place()` to guarantee fast and asymptotically stable transient recovery.
5. **Full-Order Luenberger Observer:** Designing observer gain matrix $L$ from single-sensor output $y = Cx$ to reconstruct full state vector $\hat{x}(t)$ under sensor noise.
6. **Separation Principle:** Decoupled design of controller eigenvalues and observer eigenvalues.

---

## 2. Mathematical System Model

### A. Physical Parameters
| Parameter | Symbol | Nominal Value | Unit |
| :--- | :---: | :---: | :---: |
| Cart Mass | $M$ | $1.0$ | $\text{kg}$ |
| Pendulum Mass | $m$ | $0.1$ | $\text{kg}$ |
| COM Length | $l$ | $0.5$ | $\text{m}$ |
| Gravitational Acceleration | $g$ | $9.81$ | $\text{m/s}^2$ |
| Cart Damping Friction | $b$ | $0.1$ | $\text{N}\cdot\text{s/m}$ |
| Simulation Time Step | $dt$ | $0.05$ | $\text{s}$ ($20\text{ Hz}$) |

State vector:
$$x(t) = \begin{bmatrix} p(t) \\ \dot{p}(t) \\ \theta(t) \\ \dot{\theta}(t) \end{bmatrix} \in \mathbb{R}^4$$
- $p$: Cart position ($m$)
- $\dot{p}$: Cart velocity ($m/s$)
- $\theta$: Angle from upright vertical ($rad$, positive clockwise)
- $\dot{\theta}$: Angular velocity ($rad/s$)

---

### B. Nonlinear Equations of Motion (Euler-Lagrange)
The kinetic energy $T$ and potential energy $V$ of the cart-pendulum are:
$$T = \frac{1}{2}(M + m)\dot{p}^2 + m l \dot{p} \dot{\theta} \cos\theta + \frac{1}{2} m l^2 \dot{\theta}^2$$
$$V = m g l \cos\theta$$

Applying the Euler-Lagrange equations $\frac{d}{dt}\frac{\partial L}{\partial \dot{q}_i} - \frac{\partial L}{\partial q_i} = Q_i$:
$$\begin{bmatrix}
M + m & m l \cos\theta \\
m l \cos\theta & m l^2
\end{bmatrix}
\begin{bmatrix}
\ddot{p} \\
\ddot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
u(t) + F_{\text{dist}}(t) - b \dot{p} + m l \dot{\theta}^2 \sin\theta \\
m g l \sin\theta
\end{bmatrix}$$

The determinant of the mass matrix is:
$$D(\theta) = m l^2 (M + m \sin^2\theta) > 0 \quad \forall \theta$$
Ensuring the mass matrix is strictly positive definite and invertible everywhere.

---

### C. State-Space Linearization
Linearizing around the upright equilibrium ($\theta \approx 0, \sin\theta \approx \theta, \cos\theta \approx 1, \dot{\theta}^2 \approx 0$):
$$\dot{x} = A x + B u$$
$$y = C x$$

Continuous matrices:
$$A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\frac{b}{M} & -\frac{m g}{M} & 0 \\
0 & 0 & 0 & 1 \\
0 & \frac{b}{M l} & \frac{(M+m)g}{M l} & 0
\end{bmatrix}
=
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -0.1 & -0.981 & 0 \\
0 & 0 & 0 & 1 \\
0 & 0.2 & 21.582 & 0
\end{bmatrix}$$

$$B = \begin{bmatrix}
0 \\
\frac{1}{M} \\
0 \\
-\frac{1}{M l}
\end{bmatrix}
=
\begin{bmatrix}
0 \\
1.0 \\
0 \\
-2.0
\end{bmatrix}$$

$$C = \begin{bmatrix} 1 & 0 & 0 & 0 \end{bmatrix}$$

---

### D. Pole Placement & State-Feedback Controller ($K$)
The control law is:
$$u = -K x$$

To achieve fast settling with critically damped transient response, closed-loop poles are placed at:
$$\lambda(A - BK) = [-2.0, -2.2, -1.8, -2.5]$$

Resulting state feedback gain:
$$K \approx \begin{bmatrix} -1.0092 & -2.0276 & -24.7756 & -5.2138 \end{bmatrix}$$

---

### E. Full-Order Luenberger Observer ($L$)
Only the cart position is directly measured ($y = p$). The hidden states ($\dot{p}, \theta, \dot{\theta}$) are reconstructed using:
$$\dot{\hat{x}} = A \hat{x} + B u + L(y - C \hat{x})$$

The error dynamics $e(t) = x(t) - \hat{x}(t)$ satisfy:
$$\dot{e} = (A - L C) e$$

Observer poles are placed $2.5\times$ to $3\times$ faster than plant dynamics:
$$\lambda(A - LC) = [-5.0, -5.5, -6.0, -6.5]$$

Resulting observer gain vector:
$$L \approx \begin{bmatrix} 22.90 \\ 217.04 \\ -1273.64 \\ -5914.00 \end{bmatrix}$$

---

## 3. Directory Structure

```
segway-digital-twin/
├── backend/
│   ├── main.py              # FastAPI server, CORS middleware, REST endpoints
│   ├── control_model.py     # Physics, state-space matrices, RK4 integration, K & L math
│   └── requirements.txt     # Python dependencies (fastapi, uvicorn, numpy, scipy, control)
├── frontend/
│   ├── index.html           # Dark engineering dashboard UI (Tailwind CSS)
│   ├── js/
│   │   ├── app.js           # 60 FPS Canvas loop, interactive impulse, REST fetch client
│   │   └── charts.js        # Chart.js multiseries real-time telemetry streaming
│   └── README.md            # Comprehensive project documentation & math summary
└── .gitignore               # Python/environment git ignore rules
```

---

## 4. Installation & Quickstart

### Prerequisites
- Python 3.10+ (tested on Python 3.12)
- Modern web browser (Chrome, Edge, Firefox, Safari)

### Step 1: Install Python Dependencies
Open a terminal in the project root:
```bash
cd segway-digital-twin
pip install -r backend/requirements.txt
```

### Step 2: Start the FastAPI Backend
```bash
uvicorn backend.main:app --reload --port 8000
```
The API documentation is accessible at `http://localhost:8000/docs`.

### Step 3: Launch the Frontend
You can open `frontend/index.html` directly in your browser or run a simple local web server:
```bash
cd frontend
python -m http.server 3000
```
Navigate to `http://localhost:3000` to interact with the digital twin.

---

## 5. API Reference

| Method | Endpoint | Description | Payload Example |
| :--- | :--- | :--- | :--- |
| `GET` | `/api/status` | Returns current true state $x$, estimated state $\hat{x}$, error vector, control force $u$, and stability flag. | None |
| `POST` | `/api/step` | Advances simulation by $dt = 0.05\text{s}$ using 4th-Order Runge-Kutta. | `{"force_disturbance": 15.0, "noise_enabled": true}` |
| `POST` | `/api/reset` | Resets plant to initial tilt angle (default $\theta_0 = 0.15\text{ rad}$). | `{"theta_0": 0.15, "p_0": 0.0}` |
| `GET` | `/api/model_info` | Returns matrices $A, B, C$, gain vectors $K, L$, and closed-loop eigenvalues. | None |
| `POST` | `/api/configure`| Updates control architecture (`state_feedback` vs `observer_feedback`) and noise level. | `{"control_mode": "observer_feedback"}` |

---

## 6. Frontend Controls & Shortcuts

- **Reset System (`R`):** Restores initial condition ($\theta = 0.15\text{ rad} \approx 8.6^\circ$).
- **Push Impulse (`←` / `→`):** Applies external horizontal shock force ($\pm 15\text{ N}$) to evaluate active stabilization.
- **Toggle Sensor Noise (`N`):** Injects Gaussian noise ($\sigma = 3\text{ mm}$) into cart position sensor $y = p + v$.
- **Play / Pause (`Space`):** Halts and resumes the simulation loop.
- **Math Drawer:** Click "Click to Inspect Math" in the dashboard to review live numeric matrices $A$, $B$, $K$, and $L$.
