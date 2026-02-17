"""
Motor safety monitoring module for overcurrent detection and protection.

Monitors motor current and detects fault conditions including:
- Overcurrent (exceeds rated current)
- Sustained high current (potential stall)
- Current sensor failure
"""

import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class MotorFaultType(Enum):
    """Motor fault classification"""
    NONE = "none"
    OVERCURRENT = "overcurrent"
    STALL = "stall"
    SENSOR_FAULT = "sensor_fault"
    THERMAL_LIMIT = "thermal_limit"


@dataclass
class MotorSafetyLimits:
    """Motor safety threshold parameters"""
    max_current: float = 30.0  # Amperes (36V / 1.2Ω = 30A max theoretical)
    rated_current: float = 20.0  # Amperes (safe continuous operation)
    stall_current_threshold: float = 25.0  # Amperes
    stall_duration_threshold: float = 0.5  # seconds
    current_sensor_min: float = -1.0  # Amperes (physical minimum)
    current_sensor_max: float = 35.0  # Amperes (physical maximum)
    thermal_time_constant: float = 60.0  # seconds (motor thermal time constant)
    thermal_limit: float = 1.2  # I²t thermal limit multiplier


class MotorSafetyMonitor:
    """
    Motor safety monitoring and fault detection.
    
    Provides real-time monitoring of motor current with fault detection
    for overcurrent, stall conditions, and sensor failures.
    """
    
    def __init__(self, limits: Optional[MotorSafetyLimits] = None):
        """
        Initialize motor safety monitor.
        
        Args:
            limits: Motor safety threshold parameters
        """
        self.limits = limits or MotorSafetyLimits()
        self.fault_state = MotorFaultType.NONE
        self.high_current_start_time: Optional[float] = None
        self.thermal_accumulator: float = 0.0
        self.last_update_time: Optional[float] = None
        
        logger.info(f"Motor safety monitor initialized with limits: "
                   f"max_current={self.limits.max_current}A, "
                   f"rated_current={self.limits.rated_current}A")
    
    def check_safety(self, current: float, time: float) -> Dict[str, Any]:
        """
        Check motor current against safety limits.
        
        Args:
            current: Motor current in Amperes
            time: Current simulation/system time in seconds
            
        Returns:
            Dictionary containing:
                - is_safe: bool
                - fault_type: MotorFaultType
                - message: str
                - should_shutdown: bool
        """
        # Initialize time tracking
        if self.last_update_time is None:
            self.last_update_time = time
            dt = 0.0
        else:
            dt = time - self.last_update_time
            self.last_update_time = time
        
        # Check for sensor fault (reading outside physical range)
        if not (self.limits.current_sensor_min <= current <= self.limits.current_sensor_max):
            self.fault_state = MotorFaultType.SENSOR_FAULT
            logger.error(f"Motor current sensor fault: reading {current:.2f}A "
                        f"outside valid range [{self.limits.current_sensor_min}, "
                        f"{self.limits.current_sensor_max}]")
            return {
                'is_safe': False,
                'fault_type': MotorFaultType.SENSOR_FAULT,
                'message': f"Current sensor fault: {current:.2f}A out of range",
                'should_shutdown': True
            }
        
        # Check for instantaneous overcurrent
        if current > self.limits.max_current:
            self.fault_state = MotorFaultType.OVERCURRENT
            logger.error(f"Motor overcurrent detected: {current:.2f}A > {self.limits.max_current}A")
            return {
                'is_safe': False,
                'fault_type': MotorFaultType.OVERCURRENT,
                'message': f"Overcurrent: {current:.2f}A exceeds max {self.limits.max_current}A",
                'should_shutdown': True
            }
        
        # Check for sustained high current (stall detection)
        if current > self.limits.stall_current_threshold:
            if self.high_current_start_time is None:
                self.high_current_start_time = time
                logger.warning(f"High motor current detected: {current:.2f}A")
            else:
                duration = time - self.high_current_start_time
                if duration > self.limits.stall_duration_threshold:
                    self.fault_state = MotorFaultType.STALL
                    logger.error(f"Motor stall detected: {current:.2f}A for {duration:.2f}s")
                    return {
                        'is_safe': False,
                        'fault_type': MotorFaultType.STALL,
                        'message': f"Motor stall: {current:.2f}A for {duration:.2f}s",
                        'should_shutdown': True
                    }
        else:
            self.high_current_start_time = None
        
        # Update thermal accumulator (I²t protection)
        if dt > 0:
            i_squared_normalized = (current / self.limits.rated_current) ** 2
            self.thermal_accumulator += i_squared_normalized * dt
            # Decay thermal accumulator
            decay_rate = 1.0 / self.limits.thermal_time_constant
            self.thermal_accumulator *= (1.0 - decay_rate * dt)
            
            # Check thermal limit
            if self.thermal_accumulator > self.limits.thermal_limit:
                self.fault_state = MotorFaultType.THERMAL_LIMIT
                logger.error(f"Motor thermal limit exceeded: I²t={self.thermal_accumulator:.2f}")
                return {
                    'is_safe': False,
                    'fault_type': MotorFaultType.THERMAL_LIMIT,
                    'message': f"Thermal limit exceeded: I²t={self.thermal_accumulator:.2f}",
                    'should_shutdown': True
                }
        
        # Warn if approaching rated current
        if current > self.limits.rated_current:
            logger.warning(f"Motor current {current:.2f}A exceeds rated {self.limits.rated_current}A")
        
        # All checks passed
        self.fault_state = MotorFaultType.NONE
        return {
            'is_safe': True,
            'fault_type': MotorFaultType.NONE,
            'message': "Motor operating normally",
            'should_shutdown': False
        }
    
    def reset(self):
        """Reset safety monitor state"""
        self.fault_state = MotorFaultType.NONE
        self.high_current_start_time = None
        self.thermal_accumulator = 0.0
        self.last_update_time = None
        logger.info("Motor safety monitor reset")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current safety monitor status"""
        return {
            'fault_state': self.fault_state.value,
            'thermal_accumulator': self.thermal_accumulator,
            'high_current_duration': (
                self.last_update_time - self.high_current_start_time
                if self.high_current_start_time is not None and self.last_update_time is not None
                else 0.0
            )
        }
