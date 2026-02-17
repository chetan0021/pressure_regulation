"""
Simulation package for pressure control system.

Implements high-fidelity simulation engine using scipy.integrate.

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

from .simulator import PressureControlSimulator
from .fault_simulator import FaultSimulator, SimulationMode

__all__ = ['PressureControlSimulator', 'FaultSimulator', 'SimulationMode']
