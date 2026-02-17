"""
Physical System Models Package

High-fidelity physics-based models for aerospace-grade simulation:
- DC Motor: Full electrical and mechanical state-space dynamics
- Rotary Valve: Second-order servo with Stribeck friction model
- Pressure Dynamics: Compressible fluid with bulk modulus effects

All models are configurable via config/parameters.json

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

from .motor import DCMotor
from .valve import RotaryValve
from .pressure import PressureDynamics

__all__ = ['DCMotor', 'RotaryValve', 'PressureDynamics']
