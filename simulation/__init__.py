"""
Simulation Package

Contains simulation engine for the pressure control system:
- simulator.py: Main system simulator using scipy.integrate.solve_ivp
"""

from .simulator import PressureControlSimulator

__all__ = ['PressureControlSimulator']
