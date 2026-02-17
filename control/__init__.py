"""
Control package for industrial pressure regulation.

Implements industrial-grade PID control with:
- Anti-windup (back-calculation method)
- Derivative filtering
- Output saturation handling
- Feedforward compensation
- Bumpless transfer

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

from .pid import PIDController

__all__ = ['PIDController']
