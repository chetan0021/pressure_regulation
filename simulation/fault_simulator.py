"""
Fault simulation module for injecting realistic faults into the system.

Provides:
- Leak simulation (pressure loss)
- Motor stall (torque overload)
- Valve stuck (position frozen)
- Configurable fault timing and severity
"""

import logging
import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)


class SimulationMode(Enum):
    """Simulation operating modes."""
    NOMINAL = "nominal"           # No faults
    DISTURBANCE = "disturbance"   # Pressure disturbance only
    FAULT = "fault"               # Full fault injection


@dataclass
class FaultConfig:
    """Configuration for fault injection."""
    # Leak fault
    leak_enabled: bool = False
    leak_start_time: float = 10.0  # seconds
    leak_rate: float = 1e-5  # m³/s (small leak)
    
    # Motor stall fault
    motor_stall_enabled: bool = False
    motor_stall_time: float = 15.0  # seconds
    motor_stall_duration: float = 2.0  # seconds
    
    # Valve stuck fault
    valve_stuck_enabled: bool = False
    valve_stuck_time: float = 20.0  # seconds
    valve_stuck_duration: float = 3.0  # seconds
    valve_stuck_position: Optional[float] = None  # rad, None = current position


class FaultSimulator:
    """
    Manages fault injection for system testing.
    
    Injects realistic faults to test safety systems and control robustness.
    """
    
    def __init__(self, mode: SimulationMode = SimulationMode.NOMINAL):
        """
        Initialize fault simulator.
        
        Args:
            mode: Simulation mode (NOMINAL, DISTURBANCE, or FAULT)
        """
        self.mode = mode
        self.config = FaultConfig()
        
        # Fault states
        self.leak_active = False
        self.motor_stalled = False
        self.valve_stuck = False
        self.valve_stuck_angle = None
        
        # Configure based on mode
        self._configure_mode()
        
        logger.info(f"Fault simulator initialized in {mode.value.upper()} mode")
    
    def _configure_mode(self):
        """Configure faults based on simulation mode."""
        if self.mode == SimulationMode.NOMINAL:
            # No faults
            self.config.leak_enabled = False
            self.config.motor_stall_enabled = False
            self.config.valve_stuck_enabled = False
            
        elif self.mode == SimulationMode.DISTURBANCE:
            # Only pressure disturbance (handled by simulator)
            self.config.leak_enabled = False
            self.config.motor_stall_enabled = False
            self.config.valve_stuck_enabled = False
            
        elif self.mode == SimulationMode.FAULT:
            # Full fault injection
            self.config.leak_enabled = True
            self.config.leak_start_time = 10.0
            self.config.leak_rate = 5e-6  # m³/s
            
            self.config.motor_stall_enabled = True
            self.config.motor_stall_time = 15.0
            self.config.motor_stall_duration = 2.0
            
            self.config.valve_stuck_enabled = True
            self.config.valve_stuck_time = 20.0
            self.config.valve_stuck_duration = 3.0
    
    def update(self, time: float, valve_angle: float) -> Dict[str, Any]:
        """
        Update fault states based on current time.
        
        Args:
            time: Current simulation time [s]
            valve_angle: Current valve angle [rad]
            
        Returns:
            Dictionary of active faults
        """
        # Leak fault
        if self.config.leak_enabled:
            if time >= self.config.leak_start_time and not self.leak_active:
                self.leak_active = True
                logger.warning(f"FAULT INJECTED: Leak started at t={time:.2f}s, rate={self.config.leak_rate:.2e} m³/s")
        
        # Motor stall fault
        if self.config.motor_stall_enabled:
            stall_end = self.config.motor_stall_time + self.config.motor_stall_duration
            if self.config.motor_stall_time <= time < stall_end:
                if not self.motor_stalled:
                    self.motor_stalled = True
                    logger.warning(f"FAULT INJECTED: Motor stalled at t={time:.2f}s")
            else:
                if self.motor_stalled:
                    self.motor_stalled = False
                    logger.info(f"Motor stall cleared at t={time:.2f}s")
        
        # Valve stuck fault
        if self.config.valve_stuck_enabled:
            stuck_end = self.config.valve_stuck_time + self.config.valve_stuck_duration
            if self.config.valve_stuck_time <= time < stuck_end:
                if not self.valve_stuck:
                    self.valve_stuck = True
                    self.valve_stuck_angle = valve_angle
                    logger.warning(f"FAULT INJECTED: Valve stuck at t={time:.2f}s, angle={np.rad2deg(valve_angle):.2f}°")
            else:
                if self.valve_stuck:
                    self.valve_stuck = False
                    self.valve_stuck_angle = None
                    logger.info(f"Valve unstuck at t={time:.2f}s")
        
        return self.get_fault_status()
    
    def get_leak_flow(self) -> float:
        """
        Get current leak flow rate.
        
        Returns:
            Leak flow rate [m³/s] (negative = loss)
        """
        if self.leak_active:
            return -self.config.leak_rate
        return 0.0
    
    def is_motor_stalled(self) -> bool:
        """Check if motor is currently stalled."""
        return self.motor_stalled
    
    def get_valve_override(self) -> Optional[float]:
        """
        Get valve position override if stuck.
        
        Returns:
            Stuck valve angle [rad] or None if not stuck
        """
        if self.valve_stuck:
            return self.valve_stuck_angle
        return None
    
    def get_fault_status(self) -> Dict[str, Any]:
        """
        Get current fault status.
        
        Returns:
            Dictionary with fault states
        """
        return {
            'mode': self.mode.value,
            'leak_active': self.leak_active,
            'leak_rate': self.config.leak_rate if self.leak_active else 0.0,
            'motor_stalled': self.motor_stalled,
            'valve_stuck': self.valve_stuck,
            'valve_stuck_angle': self.valve_stuck_angle
        }
    
    def reset(self):
        """Reset all fault states."""
        self.leak_active = False
        self.motor_stalled = False
        self.valve_stuck = False
        self.valve_stuck_angle = None
        logger.info("Fault simulator reset")
