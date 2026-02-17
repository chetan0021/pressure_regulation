"""
Pressure safety monitoring module for limit detection and protection.

Monitors system pressure and detects fault conditions including:
- Overpressure (exceeds maximum safe pressure)
- Underpressure (below minimum safe pressure)
- Rapid pressure changes (potential rupture/leak)
- Pressure sensor failure
"""

import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class PressureFaultType(Enum):
    """Pressure fault classification"""
    NONE = "none"
    OVERPRESSURE = "overpressure"
    UNDERPRESSURE = "underpressure"
    RAPID_CHANGE = "rapid_change"
    SENSOR_FAULT = "sensor_fault"


@dataclass
class PressureSafetyLimits:
    """Pressure safety threshold parameters"""
    max_pressure: float = 700.0  # bar (system maximum)
    safe_max_pressure: float = 650.0  # bar (safe operating maximum)
    min_pressure: float = 0.0  # bar (absolute minimum)
    safe_min_pressure: float = 10.0  # bar (safe operating minimum)
    max_rate_of_change: float = 200.0  # bar/s (maximum safe pressure rate)
    sensor_min: float = -10.0  # bar (sensor physical minimum)
    sensor_max: float = 750.0  # bar (sensor physical maximum)


class PressureSafetyMonitor:
    """
    Pressure safety monitoring and fault detection.
    
    Provides real-time monitoring of system pressure with fault detection
    for overpressure, underpressure, rapid changes, and sensor failures.
    """
    
    def __init__(self, limits: Optional[PressureSafetyLimits] = None):
        """
        Initialize pressure safety monitor.
        
        Args:
            limits: Pressure safety threshold parameters
        """
        self.limits = limits or PressureSafetyLimits()
        self.fault_state = PressureFaultType.NONE
        self.last_pressure: Optional[float] = None
        self.last_time: Optional[float] = None
        
        logger.info(f"Pressure safety monitor initialized with limits: "
                   f"max={self.limits.safe_max_pressure} bar, "
                   f"min={self.limits.safe_min_pressure} bar")
    
    def check_safety(self, pressure: float, time: float) -> Dict[str, Any]:
        """
        Check pressure against safety limits.
        
        Args:
            pressure: System pressure in bar
            time: Current simulation/system time in seconds
            
        Returns:
            Dictionary containing:
                - is_safe: bool
                - fault_type: PressureFaultType
                - message: str
                - should_shutdown: bool
        """
        # Check for sensor fault (reading outside physical range)
        if not (self.limits.sensor_min <= pressure <= self.limits.sensor_max):
            self.fault_state = PressureFaultType.SENSOR_FAULT
            logger.error(f"Pressure sensor fault: reading {pressure:.2f} bar "
                        f"outside valid range [{self.limits.sensor_min}, "
                        f"{self.limits.sensor_max}]")
            return {
                'is_safe': False,
                'fault_type': PressureFaultType.SENSOR_FAULT,
                'message': f"Pressure sensor fault: {pressure:.2f} bar out of range",
                'should_shutdown': True
            }
        
        # Check for overpressure (critical)
        if pressure > self.limits.max_pressure:
            self.fault_state = PressureFaultType.OVERPRESSURE
            logger.error(f"Critical overpressure: {pressure:.2f} bar > {self.limits.max_pressure} bar")
            return {
                'is_safe': False,
                'fault_type': PressureFaultType.OVERPRESSURE,
                'message': f"Critical overpressure: {pressure:.2f} bar",
                'should_shutdown': True
            }
        
        # Check for overpressure (warning)
        if pressure > self.limits.safe_max_pressure:
            logger.warning(f"Pressure approaching limit: {pressure:.2f} bar > "
                          f"{self.limits.safe_max_pressure} bar")
        
        # Check for underpressure
        if pressure < self.limits.min_pressure:
            self.fault_state = PressureFaultType.UNDERPRESSURE
            logger.error(f"Underpressure fault: {pressure:.2f} bar < {self.limits.min_pressure} bar")
            return {
                'is_safe': False,
                'fault_type': PressureFaultType.UNDERPRESSURE,
                'message': f"Underpressure: {pressure:.2f} bar",
                'should_shutdown': True
            }
        
        # Check for rapid pressure change (potential rupture or leak)
        if self.last_pressure is not None and self.last_time is not None:
            dt = time - self.last_time
            if dt > 0:
                rate_of_change = abs(pressure - self.last_pressure) / dt
                if rate_of_change > self.limits.max_rate_of_change:
                    self.fault_state = PressureFaultType.RAPID_CHANGE
                    logger.error(f"Rapid pressure change detected: {rate_of_change:.2f} bar/s > "
                               f"{self.limits.max_rate_of_change} bar/s")
                    return {
                        'is_safe': False,
                        'fault_type': PressureFaultType.RAPID_CHANGE,
                        'message': f"Rapid pressure change: {rate_of_change:.2f} bar/s",
                        'should_shutdown': True
                    }
        
        # Update history
        self.last_pressure = pressure
        self.last_time = time
        
        # All checks passed
        self.fault_state = PressureFaultType.NONE
        return {
            'is_safe': True,
            'fault_type': PressureFaultType.NONE,
            'message': "Pressure within safe limits",
            'should_shutdown': False
        }
    
    def reset(self):
        """Reset safety monitor state"""
        self.fault_state = PressureFaultType.NONE
        self.last_pressure = None
        self.last_time = None
        logger.info("Pressure safety monitor reset")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current safety monitor status"""
        return {
            'fault_state': self.fault_state.value,
            'last_pressure': self.last_pressure,
            'rate_of_change': (
                abs(self.last_pressure - self.last_pressure) / 
                (self.last_time - self.last_time)
                if self.last_pressure is not None and self.last_time is not None
                else 0.0
            )
        }
