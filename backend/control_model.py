"""
Digital Twin of a Segway / Inverted Pendulum
Control Model: Physical Dynamics, State-Space Linearization, Pole Placement, and Luenberger Observer.
"""

import os
import tempfile
# Ensure Matplotlib uses a valid writable temp directory silently
os.environ.setdefault("MPLCONFIGDIR", tempfile.gettempdir())

from typing import Dict, Any, List, Optional
import numpy as np

try:
    import control
    HAS_CONTROL = True
except ImportError:
    HAS_CONTROL = False

try:
    from scipy.signal import place_poles
    from scipy.integrate import solve_ivp
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


class SegwayControlModel:
    """
    Mathematical and Simulation Model for the Cart-Pendulum (Segway) System.
    
    States:
        x[0] = p        : Cart horizontal position (m)
        x[1] = p_dot    : Cart horizontal velocity (m/s)
        x[2] = theta    : Pendulum angle from upright vertical (rad, positive clockwise/right)
        x[3] = theta_dot: Pendulum angular velocity (rad/s)
    """

    def __init__(
        self,
        M: float = 1.0,      # Mass of cart (kg)
        m: float = 0.1,      # Mass of pendulum (kg)
        l: float = 0.5,      # Length to pendulum center of mass (m)
        g: float = 9.81,     # Gravitational acceleration (m/s^2)
        b: float = 0.1,      # Cart friction coefficient (N/m/s)
        dt: float = 0.05,    # Simulation time step (s)
        ctrl_poles: Optional[List[float]] = None,
        obs_poles: Optional[List[float]] = None
    ):
        self.M = float(M)
        self.m = float(m)
        self.l = float(l)
        self.g = float(g)
        self.b = float(b)
        self.dt = float(dt)
        self.I = 0.0         # Moment of inertia around COM (0 for point mass bob)

        # Pole placement targets
        # Fast, stable, well-damped transient response
        self.ctrl_poles = ctrl_poles or [-2.0, -2.2, -1.8, -2.5]
        # Observer poles placed faster than plant (approx 2.5x to 3x faster)
        self.obs_poles = obs_poles or [-5.0, -5.5, -6.0, -6.5]

        # Matrices
        self.A = np.zeros((4, 4), dtype=float)
        self.B = np.zeros((4, 1), dtype=float)
        self.C = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=float)
        self.K = np.zeros((1, 4), dtype=float)
        self.L = np.zeros((4, 1), dtype=float)

        # Build state-space and gains
        self._build_linear_model()
        self._compute_gains()

        # Physical and Observer State vectors
        # True state: [p, p_dot, theta, theta_dot]
        self.x = np.array([0.0, 0.0, 0.15, 0.0], dtype=float)  # default initial tilt 0.15 rad (~8.6 deg)
        # Observer estimated state: [p_hat, p_dot_hat, theta_hat, theta_dot_hat]
        self.x_hat = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)

        # Simulation telemetry
        self.time = 0.0
        self.u = 0.0
        self.current_disturbance = 0.0
        self.measurement_noise_enabled = False
        self.noise_std = 0.003  # 3 mm sensor noise on position
        self.last_measurement_y = 0.0
        self.control_mode = "state_feedback"  # "state_feedback" (u=-Kx) or "observer_feedback" (u=-K*x_hat)
        self.actuator_limit = 60.0  # Max force saturation in Newtons

    def _build_linear_model(self) -> None:
        """
        Linearize continuous-time dynamics around upright equilibrium theta = 0.
        
        Derivation from Euler-Lagrange:
            (M + m) * p_ddot + m * l * theta_ddot = u - b * p_dot
            m * l * p_ddot + (I + m * l^2) * theta_ddot = m * g * l * theta
            
        Solving for [p_ddot, theta_ddot]^T with I = 0 (point mass bob):
            A = [
                [0, 1, 0, 0],
                [0, -b/M, -m*g/M, 0],
                [0, 0, 0, 1],
                [0, b/(M*l), (M+m)*g/(M*l), 0]
            ]
            B = [
                [0],
                [1/M],
                [0],
                [-1/(M*l)]
            ]
        """
        M, m, l, g, b = self.M, self.m, self.l, self.g, self.b
        
        self.A = np.array([
            [0.0, 1.0, 0.0, 0.0],
            [0.0, -b / M, -(m * g) / M, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, b / (M * l), ((M + m) * g) / (M * l), 0.0]
        ], dtype=float)

        self.B = np.array([
            [0.0],
            [1.0 / M],
            [0.0],
            [-1.0 / (M * l)]
        ], dtype=float)

        self.C = np.array([[1.0, 0.0, 0.0, 0.0]], dtype=float)

    def _compute_gains(self) -> None:
        """
        Compute feedback gain K and observer gain L via pole placement.
        """
        # 1. State Feedback Gain K (u = -Kx)
        if HAS_CONTROL:
            try:
                self.K = np.array(control.place(self.A, self.B, self.ctrl_poles), dtype=float)
            except Exception:
                self.K = self._place_poles_scipy_or_ackermann(self.A, self.B, self.ctrl_poles)
        elif HAS_SCIPY:
            self.K = self._place_poles_scipy_or_ackermann(self.A, self.B, self.ctrl_poles)
        else:
            self.K = self._ackermann(self.A, self.B, self.ctrl_poles)

        # 2. Luenberger Observer Gain L (dot(x_hat) = A*x_hat + B*u + L*(y - C*x_hat))
        # By duality, poles of (A - LC) are the eigenvalues of (A^T - C^T L^T)
        if HAS_CONTROL:
            try:
                L_T = control.place(self.A.T, self.C.T, self.obs_poles)
                self.L = np.array(L_T.T, dtype=float)
            except Exception:
                self.L = self._place_poles_dual(self.A, self.C, self.obs_poles)
        elif HAS_SCIPY:
            self.L = self._place_poles_dual(self.A, self.C, self.obs_poles)
        else:
            self.L = self._ackermann_observer(self.A, self.C, self.obs_poles)

    def _place_poles_scipy_or_ackermann(self, A: np.ndarray, B: np.ndarray, poles: List[float]) -> np.ndarray:
        if HAS_SCIPY:
            res = place_poles(A, B, poles)
            return res.gain_matrix
        return self._ackermann(A, B, poles)

    def _place_poles_dual(self, A: np.ndarray, C: np.ndarray, poles: List[float]) -> np.ndarray:
        if HAS_SCIPY:
            res = place_poles(A.T, C.T, poles)
            return res.gain_matrix.T
        return self._ackermann_observer(A, C, poles)

    def _ackermann(self, A: np.ndarray, B: np.ndarray, poles: List[float]) -> np.ndarray:
        """Ackermann's formula for single-input controllable systems: K = [0 ... 1] * C_inv * phi(A)"""
        n = A.shape[0]
        # Controllability matrix
        Ctr = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(n)])
        # Desired characteristic polynomial: phi(s) = (s - p1)(s - p2)...
        poly = np.poly(poles)
        phi_A = sum(poly[i] * np.linalg.matrix_power(A, n - i) for i in range(n + 1))
        e_n = np.zeros((1, n))
        e_n[0, -1] = 1.0
        K = e_n @ np.linalg.inv(Ctr) @ phi_A
        return K

    def _ackermann_observer(self, A: np.ndarray, C: np.ndarray, poles: List[float]) -> np.ndarray:
        """Ackermann formula for observer gain L via dual system."""
        n = A.shape[0]
        Obs = np.vstack([C @ np.linalg.matrix_power(A, i) for i in range(n)])
        poly = np.poly(poles)
        phi_A = sum(poly[i] * np.linalg.matrix_power(A, n - i) for i in range(n + 1))
        e_n = np.zeros((n, 1))
        e_n[-1, 0] = 1.0
        L = phi_A @ np.linalg.inv(Obs) @ e_n
        return L

    def nonlinear_derivatives(self, state: np.ndarray, u_input: float, disturbance: float = 0.0) -> np.ndarray:
        """
        Compute true nonlinear derivatives dx/dt = [p_dot, p_ddot, theta_dot, theta_ddot].
        
        Dynamic Equations:
            [M + m           m * l * cos(theta)   ] [p_ddot    ]   [u + F_dist - b*p_dot + m*l*theta_dot^2*sin(theta)]
            [m * l * cos(th) m * l^2              ] [theta_ddot] = [m * g * l * sin(theta)                           ]
        """
        p, p_dot, theta, theta_dot = state
        cos_th = np.cos(theta)
        sin_th = np.sin(theta)

        M, m, l, g, b = self.M, self.m, self.l, self.g, self.b
        
        # Mass matrix elements
        M11 = M + m
        M12 = m * l * cos_th
        M21 = m * l * cos_th
        M22 = m * (l ** 2)

        det = M11 * M22 - M12 * M21  # = m * l^2 * (M + m * sin(theta)^2) > 0 for all theta

        # Right hand side forces
        rhs1 = u_input + disturbance - b * p_dot + m * l * (theta_dot ** 2) * sin_th
        rhs2 = m * g * l * sin_th

        # Exact inversion of 2x2 mass matrix
        p_ddot = (M22 * rhs1 - M12 * rhs2) / det
        theta_ddot = (-M21 * rhs1 + M11 * rhs2) / det

        return np.array([p_dot, p_ddot, theta_dot, theta_ddot], dtype=float)

    def observer_derivatives(self, x_hat: np.ndarray, u_input: float, y_meas: float) -> np.ndarray:
        """
        Continuous Luenberger observer derivative:
            dot(x_hat) = A * x_hat + B * u + L * (y - C * x_hat)
        """
        y_est = float(self.C.dot(x_hat)[0])
        innov = y_meas - y_est
        dx_hat = (self.A @ x_hat) + (self.B.flatten() * u_input) + (self.L.flatten() * innov)
        return dx_hat

    def step(
        self,
        force_disturbance: float = 0.0,
        noise_enabled: Optional[bool] = None,
        noise_std: Optional[float] = None
    ) -> Dict[str, Any]:
        """
        Advance the digital twin simulation by dt (0.05 s).
        """
        if noise_enabled is not None:
            self.measurement_noise_enabled = noise_enabled
        if noise_std is not None:
            self.noise_std = max(0.0, float(noise_std))

        self.current_disturbance = float(force_disturbance)

        # 1. Determine Control Force Input
        # Choice of feedback state: Full true state or Observer estimated state
        if self.control_mode == "observer_feedback":
            u_calc = -float(self.K.dot(self.x_hat)[0])
        else:
            u_calc = -float(self.K.dot(self.x)[0])

        # Apply actuator saturation limits
        self.u = float(np.clip(u_calc, -self.actuator_limit, self.actuator_limit))

        # 2. Advance True Physical Plant using 4th-Order Runge-Kutta (RK4)
        dt = self.dt
        u_total = self.u
        dist = self.current_disturbance

        k1 = self.nonlinear_derivatives(self.x, u_total, dist)
        k2 = self.nonlinear_derivatives(self.x + 0.5 * dt * k1, u_total, dist)
        k3 = self.nonlinear_derivatives(self.x + 0.5 * dt * k2, u_total, dist)
        k4 = self.nonlinear_derivatives(self.x + dt * k3, u_total, dist)
        self.x = self.x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # 3. Simulate Sensor Measurement: y = C * x + v (noise)
        raw_position = float(self.x[0])
        if self.measurement_noise_enabled:
            noise_val = float(np.random.normal(0.0, self.noise_std))
        else:
            noise_val = 0.0
        y_meas = raw_position + noise_val
        self.last_measurement_y = y_meas

        # 4. Advance Luenberger Observer using RK4
        k1_obs = self.observer_derivatives(self.x_hat, self.u, y_meas)
        k2_obs = self.observer_derivatives(self.x_hat + 0.5 * dt * k1_obs, self.u, y_meas)
        k3_obs = self.observer_derivatives(self.x_hat + 0.5 * dt * k2_obs, self.u, y_meas)
        k4_obs = self.observer_derivatives(self.x_hat + dt * k3_obs, self.u, y_meas)
        self.x_hat = self.x_hat + (dt / 6.0) * (k1_obs + 2.0 * k2_obs + 2.0 * k3_obs + k4_obs)

        # Update elapsed simulation time
        self.time += dt

        return self.get_status()

    def reset(
        self,
        p_0: float = 0.0,
        theta_0: float = 0.15,
        p_dot_0: float = 0.0,
        theta_dot_0: float = 0.0,
        reset_observer_to_zero: bool = True
    ) -> Dict[str, Any]:
        """
        Reset the physical simulation state back to a tilted configuration.
        """
        self.x = np.array([float(p_0), float(p_dot_0), float(theta_0), float(theta_dot_0)], dtype=float)
        if reset_observer_to_zero:
            self.x_hat = np.zeros(4, dtype=float)
        else:
            self.x_hat = self.x.copy()

        self.time = 0.0
        self.u = 0.0
        self.current_disturbance = 0.0
        self.last_measurement_y = float(self.x[0])

        return self.get_status()

    def get_status(self) -> Dict[str, Any]:
        """
        Returns full telemetry packet matching API specifications.
        """
        error_vec = (self.x - self.x_hat).tolist()
        theta_rad = float(self.x[2])
        # Upright stability criterion: angle within 0.35 rad (~20 deg)
        is_stable = bool(abs(theta_rad) < 0.35)

        return {
            "time": round(self.time, 3),
            "state": {
                "p": round(float(self.x[0]), 5),
                "p_dot": round(float(self.x[1]), 5),
                "theta": round(float(self.x[2]), 5),
                "theta_dot": round(float(self.x[3]), 5),
                "theta_deg": round(float(np.degrees(self.x[2])), 3)
            },
            "estimated_state": {
                "p": round(float(self.x_hat[0]), 5),
                "p_dot": round(float(self.x_hat[1]), 5),
                "theta": round(float(self.x_hat[2]), 5),
                "theta_dot": round(float(self.x_hat[3]), 5),
                "theta_deg": round(float(np.degrees(self.x_hat[2])), 3)
            },
            "error": [round(val, 5) for val in error_vec],
            "error_norm": round(float(np.linalg.norm(self.x - self.x_hat)), 5),
            "control_input_u": round(float(self.u), 4),
            "disturbance": round(float(self.current_disturbance), 4),
            "measurement_y": round(float(self.last_measurement_y), 5),
            "noise_enabled": bool(self.measurement_noise_enabled),
            "noise_std": float(self.noise_std),
            "control_mode": self.control_mode,
            "stable": is_stable
        }

    def get_model_info(self) -> Dict[str, Any]:
        """
        Returns continuous state-space matrices, gain vectors, and closed-loop poles.
        """
        # Closed loop poles: eig(A - B*K)
        cl_eig = np.linalg.eigvals(self.A - self.B @ self.K)
        # Observer poles: eig(A - L*C)
        obs_eig = np.linalg.eigvals(self.A - self.L @ self.C)

        return {
            "parameters": {
                "M": self.M,
                "m": self.m,
                "l": self.l,
                "g": self.g,
                "b": self.b,
                "dt": self.dt
            },
            "matrices": {
                "A": self.A.tolist(),
                "B": self.B.tolist(),
                "C": self.C.tolist()
            },
            "gains": {
                "K": self.K.tolist(),
                "L": self.L.tolist()
            },
            "desired_poles": {
                "controller": self.ctrl_poles,
                "observer": self.obs_poles
            },
            "eigenvalues": {
                "open_loop": [{"real": round(float(e.real), 4), "imag": round(float(e.imag), 4)} for e in np.linalg.eigvals(self.A)],
                "closed_loop_controller": [{"real": round(float(e.real), 4), "imag": round(float(e.imag), 4)} for e in cl_eig],
                "closed_loop_observer": [{"real": round(float(e.real), 4), "imag": round(float(e.imag), 4)} for e in obs_eig]
            }
        }
