"""
Valve safety monitoring module for position limit checks and protection.

Monitors valve position and detects fault conditions including:
- Position limits (0° to 180°)
- Excessive velocity
- Position sensor failure
- Mechanical jamming detection
"""

import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class ValveFaultType(Enum):
    """Valve fault classification"""
    NONE = "none"
    POSITION_LIMIT = "position_limit"
    EXCESSIVE_VELOCITY = "excessive_velocity"
    SENSOR_FAULT = "sensor_fault"
    JAM_DETECTED = "jam_detected"


@dataclass
class ValveSafetyLimits:
    """Valve safety threshold parameters"""
    min_angle: float = 0.0  # degrees
    max_angle: float = 180.0  # degrees
    max_velocity: float = 90.0  # degrees/second (safe maximum)
    sensor_min: float = -10.0  # degrees (sensor physical minimum)
    sensor_max: float = 190.0  # degrees (sensor physical maximum)
    jam_velocity_threshold: float = 1.0  # degrees/second (minimum expected velocity)
    jam_detection_time: float = 2.0  # seconds (time to detect jam)


class ValveSafetyMonitor:
    """
    Valve safety monitoring and fault detection.
    
    Provides real-time monitoring of valve position with fault detection
    for position limits, excessive velocity, and sensor failures.
    """
    
    def __init__(self, limits: Optional[ValveSafetyLimits] = None):
        """
        Initialize valve safety monitor.
        
        Args:
            limits: Valve safety threshold parameters
        """
        self.limits = limits or ValveSafetyLimits()
        self.fault_state = ValveFaultType.NONE
        self.last_angle: Optional[float] = None
        self.last_time: Optional[float] = None
        self.low_velocity_start_time: Optional[float] = None
        
        logger.info(f"Valve safety monitor initialized with limits: "
                   f"angle=[{self.limits.min_angle}°, {self.limits.max_angle}°], "
                   f"max_velocity={self.limits.max_velocity}°/s")
    
    def check_safety(self, angle: float, time: float, 
                    commanded_velocity: Optional[float] = None) -> Dict[str, Any]:
        """
        Check valve position against safety limits.
        
        Args:
            angle: Valve angle in degrees
            time: Current simulation/system time in seconds
            commanded_velocity: Commanded velocity in degrees/second (optional, for jam detection)
            
        Returns:
            Dictionary containing:
                - is_safe: bool
                - fault_type: ValveFaultType
                - message: str
                - should_shutdown: bool
        """
        # Check for sensor fault (reading outside physical range)
        if not (self.limits.sensor_min <= angle <= self.limits.sensor_max):
            self.fault_state = ValveFaultType.SENSOR_FAULT
            logger.error(f"Valve position sensor fault: reading {angle:.2f}° "
                        f"outside valid range [{self.limits.sensor_min}°, "
                        f"{self.limits.sensor_max}°]")
            return {
                'is_safe': False,
                'fault_type': ValveFaultType.SENSOR_FAULT,
                'message': f"Valve sensor fault: {angle:.2f}° out of range",
                'should_shutdown': True
            }
        
        # Check for position limits
        if angle < self.limits.min_angle or angle > self.limits.max_angle:
            self.fault_state = ValveFaultType.POSITION_LIMIT
            logger.error(f"Valve position limit exceeded: {angle:.2f}° "
                        f"outside [{self.limits.min_angle}°, {self.limits.max_angle}°]")
            return {
                'is_safe': False,
                'fault_type': ValveFaultType.POSITION_LIMIT,
                'message': f"Valve position limit: {angle:.2f}° out of bounds",
                'should_shutdown': True
            }
        
        # Check for excessive velocity
        if self.last_angle is not None and self.last_time is not None:
            dt = time - self.last_time
            if dt > 0:
                velocity = abs(angle - self.last_angle) / dt
                if velocity > self.limits.max_velocity:
                    self.fault_state = ValveFaultType.EXCESSIVE_VELOCITY
                    logger.error(f"Excessive valve velocity: {velocity:.2f}°/s > "
                               f"{self.limits.max_velocity}°/s")
                    return {
                        'is_safe': False,
                        'fault_type': ValveFaultType.EXCESSIVE_VELOCITY,
                        'message': f"Excessive velocity: {velocity:.2f}°/s",
                        'should_shutdown': True
                    }
                
                # Check for mechanical jamming (commanded motion but low actual velocity)
                if commanded_velocity is not None and abs(commanded_velocity) > self.limits.jam_velocity_threshold:
                    if velocity < self.limits.jam_velocity_threshold:
                        if self.low_velocity_start_time is None:
                            self.low_velocity_start_time = time
                        else:
                            jam_duration = time - self.low_velocity_start_time
                            if jam_duration > self.limits.jam_detection_time:
                                self.fault_state = ValveFaultType.JAM_DETECTED
                                logger.error(f"Valve jam detected: commanded {commanded_velocity:.2f}°/s "
                                           f"but actual {velocity:.2f}°/s for {jam_duration:.2f}s")
                                return {
                                    'is_safe': False,
                                    'fault_type': ValveFaultType.JAM_DETECTED,
                                    'message': f"Valve jam detected after {jam_duration:.2f}s",
                                    'should_shutdown': True
                                }
                    else:
                        self.low_velocity_start_time = None
        
        # Update history
        self.last_angle = angle
        self.last_time = time
        
        # All checks passed
        self.fault_state = ValveFaultType.NONE
        return {
            'is_safe': True,
            'fault_type': ValveFaultType.NONE,
            'message': "Valve operating normally",
            'should_shutdown': False
        }
    
    def reset(self):
        """Reset safety monitor state"""
        self.fault_state = ValveFaultType.NONE
        self.last_angle = None
        self.last_time = None
        self.low_velocity_start_time = None
        logger.info("Valve safety monitor reset")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current safety monitor status"""
        return {
            'fault_state': self.fault_state.value,
            'last_angle': self.last_angle,
            'velocity': (
                abs(self.last_angle - self.last_angle) / 
                (self.last_time - self.last_time)
                if self.last_angle is not None and self.last_time is not None
                else 0.0
            )
        }
