"""
FastAPI Server for Digital Twin of a Segway / Inverted Pendulum.
Provides REST API endpoints for simulation stepping, telemetry querying, and state reset.
"""

from typing import Optional
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field

from control_model import SegwayControlModel

app = FastAPI(
    title="Digital Twin of Inverted Pendulum API",
    description="Real-time control systems simulation with State Feedback & Observer",
    version="1.0.0"
)

# Enable CORS for frontend integration
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global simulation instance
simulator = SegwayControlModel()


class StepRequest(BaseModel):
    force_disturbance: Optional[float] = Field(0.0, description="External push impulse force in Newtons")
    noise_enabled: Optional[bool] = Field(None, description="Toggle measurement noise")
    noise_std: Optional[float] = Field(None, description="Gaussian noise standard deviation (meters)")


class ResetRequest(BaseModel):
    p_0: Optional[float] = Field(0.0, description="Initial cart position (m)")
    theta_0: Optional[float] = Field(0.15, description="Initial pendulum angle (rad)")
    p_dot_0: Optional[float] = Field(0.0, description="Initial cart velocity (m/s)")
    theta_dot_0: Optional[float] = Field(0.0, description="Initial angular velocity (rad/s)")
    reset_observer: Optional[bool] = Field(True, description="Reset observer estimate to zero")


class ConfigRequest(BaseModel):
    control_mode: Optional[str] = Field(None, description="'state_feedback' or 'observer_feedback'")
    noise_enabled: Optional[bool] = Field(None, description="Enable or disable sensor noise")
    noise_std: Optional[float] = Field(None, description="Sensor noise standard deviation")
    actuator_limit: Optional[float] = Field(None, description="Actuator force limit saturation (N)")


@app.get("/")
def root():
    return {
        "name": "Digital Twin Segway / Inverted Pendulum API",
        "status": "online",
        "documentation": "/docs",
        "endpoints": [
            "GET /api/status",
            "POST /api/step",
            "POST /api/reset",
            "GET /api/model_info",
            "POST /api/configure"
        ]
    }


@app.get("/health")
def health():
    return {"status": "healthy"}


@app.get("/api/status")
def get_status():
    """
    Returns current true state x, estimated state x_hat, error vector, control input u,
    and stability flag (stable: true/false).
    """
    return simulator.get_status()


@app.post("/api/step")
def step_simulation(payload: StepRequest = StepRequest()):
    """
    Advances the simulation by fixed time step dt = 0.05s using RK4 / ODE solver.
    Accepts an optional JSON body parameter for external push impulse force.
    """
    try:
        result = simulator.step(
            force_disturbance=payload.force_disturbance or 0.0,
            noise_enabled=payload.noise_enabled,
            noise_std=payload.noise_std
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/reset")
def reset_simulation(payload: ResetRequest = ResetRequest()):
    """
    Resets the physical simulation state back to a tilted configuration (default theta = 0.15 rad).
    """
    try:
        result = simulator.reset(
            p_0=payload.p_0 if payload.p_0 is not None else 0.0,
            theta_0=payload.theta_0 if payload.theta_0 is not None else 0.15,
            p_dot_0=payload.p_dot_0 if payload.p_dot_0 is not None else 0.0,
            theta_dot_0=payload.theta_dot_0 if payload.theta_dot_0 is not None else 0.0,
            reset_observer_to_zero=payload.reset_observer if payload.reset_observer is not None else True
        )
        return result
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.get("/api/model_info")
def get_model_info():
    """
    Returns physical parameters, state-space matrices (A, B, C),
    controller gain K, observer gain L, and eigenvalue poles.
    """
    try:
        return simulator.get_model_info()
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@app.post("/api/configure")
def configure_model(payload: ConfigRequest):
    """
    Update runtime control parameters (e.g., feedback mode, noise parameters).
    """
    if payload.control_mode in ["state_feedback", "observer_feedback"]:
        simulator.control_mode = payload.control_mode
    if payload.noise_enabled is not None:
        simulator.measurement_noise_enabled = payload.noise_enabled
    if payload.noise_std is not None:
        simulator.noise_std = max(0.0, payload.noise_std)
    if payload.actuator_limit is not None:
        simulator.actuator_limit = max(1.0, payload.actuator_limit)
    return simulator.get_status()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("backend.main:app", host="0.0.0.0", port=8000, reload=True)
