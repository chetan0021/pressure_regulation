"""
Sensor validation module for range checking and fault detection.

Provides validation for all system sensors with:
- Physical range checking
- Fallback behavior on sensor failure
- Sensor health monitoring
"""

import logging
from typing import Optional, Dict, Any, Callable
from dataclasses import dataclass
from enum import Enum

logger = logging.getLogger(__name__)


class SensorStatus(Enum):
    """Sensor health status"""
    HEALTHY = "healthy"
    OUT_OF_RANGE = "out_of_range"
    FAILED = "failed"
    FALLBACK = "fallback"


@dataclass
class SensorLimits:
    """Sensor physical limits and validation parameters"""
    min_value: float
    max_value: float
    name: str
    units: str
    fallback_value: Optional[float] = None


class SensorValidator:
    """
    Generic sensor validator with range checking and fallback.
    
    Validates sensor readings against physical limits and provides
    fallback behavior on sensor failure.
    """
    
    def __init__(self, limits: SensorLimits):
        """
        Initialize sensor validator.
        
        Args:
            limits: Sensor limits and configuration
        """
        self.limits = limits
        self.status = SensorStatus.HEALTHY
        self.consecutive_failures = 0
        self.max_consecutive_failures = 3
        self.last_valid_reading: Optional[float] = None
        
        logger.info(f"Sensor validator initialized for {limits.name}: "
                   f"range=[{limits.min_value}, {limits.max_value}] {limits.units}")
    
    def validate(self, reading: float) -> Dict[str, Any]:
        """
        Validate sensor reading.
        
        Args:
            reading: Sensor reading value
            
        Returns:
            Dictionary containing:
                - is_valid: bool
                - validated_value: float (original or fallback)
                - status: SensorStatus
                - message: str
        """
        # Check if reading is within physical range
        if self.limits.min_value <= reading <= self.limits.max_value:
            # Valid reading
            self.status = SensorStatus.HEALTHY
            self.consecutive_failures = 0
            self.last_valid_reading = reading
            
            return {
                'is_valid': True,
                'validated_value': reading,
                'status': SensorStatus.HEALTHY,
                'message': f"{self.limits.name} reading valid: {reading:.2f} {self.limits.units}"
            }
        else:
            # Out of range reading
            self.consecutive_failures += 1
            
            logger.warning(f"{self.limits.name} out of range: {reading:.2f} {self.limits.units} "
                          f"(valid range: [{self.limits.min_value}, {self.limits.max_value}])")
            
            # Check if sensor has failed (multiple consecutive failures)
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.status = SensorStatus.FAILED
                logger.error(f"{self.limits.name} sensor FAILED after "
                           f"{self.consecutive_failures} consecutive out-of-range readings")
                
                # Use fallback value if available
                if self.limits.fallback_value is not None:
                    self.status = SensorStatus.FALLBACK
                    logger.warning(f"{self.limits.name} using fallback value: "
                                 f"{self.limits.fallback_value} {self.limits.units}")
                    return {
                        'is_valid': False,
                        'validated_value': self.limits.fallback_value,
                        'status': SensorStatus.FALLBACK,
                        'message': f"{self.limits.name} sensor failed - using fallback"
                    }
                elif self.last_valid_reading is not None:
                    self.status = SensorStatus.FALLBACK
                    logger.warning(f"{self.limits.name} using last valid reading: "
                                 f"{self.last_valid_reading} {self.limits.units}")
                    return {
                        'is_valid': False,
                        'validated_value': self.last_valid_reading,
                        'status': SensorStatus.FALLBACK,
                        'message': f"{self.limits.name} sensor failed - using last valid reading"
                    }
                else:
                    return {
                        'is_valid': False,
                        'validated_value': 0.0,
                        'status': SensorStatus.FAILED,
                        'message': f"{self.limits.name} sensor failed - no fallback available"
                    }
            else:
                self.status = SensorStatus.OUT_OF_RANGE
                # Use last valid reading for temporary out-of-range
                fallback = self.last_valid_reading if self.last_valid_reading is not None else 0.0
                return {
                    'is_valid': False,
                    'validated_value': fallback,
                    'status': SensorStatus.OUT_OF_RANGE,
                    'message': f"{self.limits.name} temporarily out of range ({self.consecutive_failures}/{self.max_consecutive_failures})"
                }
    
    def reset(self):
        """Reset sensor validator state"""
        self.status = SensorStatus.HEALTHY
        self.consecutive_failures = 0
        logger.info(f"{self.limits.name} sensor validator reset")
    
    def get_status(self) -> Dict[str, Any]:
        """Get current sensor status"""
        return {
            'sensor_name': self.limits.name,
            'status': self.status.value,
            'consecutive_failures': self.consecutive_failures,
            'last_valid_reading': self.last_valid_reading
        }


class SystemSensorValidators:
    """
    Collection of all system sensor validators.
    
    Provides centralized sensor validation for the entire system.
    """
    
    def __init__(self):
        """Initialize all system sensor validators"""
        # Pressure sensor (0-750 bar physical range)
        self.pressure = SensorValidator(SensorLimits(
            min_value=-10.0,
            max_value=750.0,
            name="Pressure",
            units="bar",
            fallback_value=100.0  # Safe fallback pressure
        ))
        
        # Motor current sensor (-1 to 35 A physical range)
        self.motor_current = SensorValidator(SensorLimits(
            min_value=-1.0,
            max_value=35.0,
            name="Motor Current",
            units="A",
            fallback_value=0.0  # Safe fallback (motor off)
        ))
        
        # Valve position sensor (-10° to 190° physical range)
        self.valve_position = SensorValidator(SensorLimits(
            min_value=-10.0,
            max_value=190.0,
            name="Valve Position",
            units="degrees",
            fallback_value=90.0  # Safe fallback (mid-position)
        ))
        
        logger.info("System sensor validators initialized")
    
    def validate_all(self, pressure: float, current: float, angle: float) -> Dict[str, Any]:
        """
        Validate all sensor readings.
        
        Args:
            pressure: Pressure sensor reading (bar)
            current: Motor current sensor reading (A)
            angle: Valve position sensor reading (degrees)
            
        Returns:
            Dictionary with validated values and status for all sensors
        """
        pressure_result = self.pressure.validate(pressure)
        current_result = self.motor_current.validate(current)
        angle_result = self.valve_position.validate(angle)
        
        all_valid = (
            pressure_result['is_valid'] and
            current_result['is_valid'] and
            angle_result['is_valid']
        )
        
        return {
            'all_valid': all_valid,
            'pressure': pressure_result,
            'motor_current': current_result,
            'valve_position': angle_result,
            'validated_values': {
                'pressure': pressure_result['validated_value'],
                'motor_current': current_result['validated_value'],
                'valve_position': angle_result['validated_value']
            }
        }
    
    def reset_all(self):
        """Reset all sensor validators"""
        self.pressure.reset()
        self.motor_current.reset()
        self.valve_position.reset()
        logger.info("All sensor validators reset")
    
    def get_all_status(self) -> Dict[str, Any]:
        """Get status of all sensors"""
        return {
            'pressure': self.pressure.get_status(),
            'motor_current': self.motor_current.get_status(),
            'valve_position': self.valve_position.get_status()
        }
