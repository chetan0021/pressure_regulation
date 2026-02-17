"""
Control Package

Contains control algorithms for the pressure regulation system:
- pid.py: PID controller implementation
"""

from .pid import PIDController

__all__ = ['PIDController']
