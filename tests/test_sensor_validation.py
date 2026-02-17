"""
Unit tests for sensor validation module.
"""

import pytest
from validation.sensor_validation import (
    SensorValidator,
    SystemSensorValidators,
    SensorStatus,
    SensorLimits
)


class TestSensorValidator:
    """Test suite for sensor validation"""
    
    def test_initialization(self):
        """Test sensor validator initialization"""
        limits = SensorLimits(
            min_value=0.0,
            max_value=100.0,
            name="Test Sensor",
            units="units"
        )
        validator = SensorValidator(limits)
        
        assert validator.status == SensorStatus.HEALTHY
        assert validator.consecutive_failures == 0
    
    def test_valid_reading(self):
        """Test validation of valid sensor reading"""
        limits = SensorLimits(min_value=0.0, max_value=100.0, name="Test", units="units")
        validator = SensorValidator(limits)
        
        result = validator.validate(50.0)
        
        assert result['is_valid'] is True
        assert result['validated_value'] == 50.0
        assert result['status'] == SensorStatus.HEALTHY
    
    def test_out_of_range_reading(self):
        """Test validation of out-of-range reading"""
        limits = SensorLimits(min_value=0.0, max_value=100.0, name="Test", units="units")
        validator = SensorValidator(limits)
        
        result = validator.validate(150.0)
        
        assert result['is_valid'] is False
        assert result['status'] == SensorStatus.OUT_OF_RANGE
    
    def test_sensor_failure_detection(self):
        """Test sensor failure after multiple consecutive out-of-range readings"""
        limits = SensorLimits(
            min_value=0.0,
            max_value=100.0,
            name="Test",
            units="units",
            fallback_value=50.0
        )
        validator = SensorValidator(limits)
        
        # First out-of-range reading
        result = validator.validate(150.0)
        assert result['status'] == SensorStatus.OUT_OF_RANGE
        
        # Second out-of-range reading
        result = validator.validate(150.0)
        assert result['status'] == SensorStatus.OUT_OF_RANGE
        
        # Third out-of-range reading - should trigger failure
        result = validator.validate(150.0)
        assert result['status'] == SensorStatus.FALLBACK
        assert result['validated_value'] == 50.0
    
    def test_fallback_to_last_valid(self):
        """Test fallback to last valid reading when no fallback value specified"""
        limits = SensorLimits(min_value=0.0, max_value=100.0, name="Test", units="units")
        validator = SensorValidator(limits)
        
        # Valid reading
        validator.validate(75.0)
        
        # Multiple out-of-range readings
        for _ in range(3):
            result = validator.validate(150.0)
        
        assert result['status'] == SensorStatus.FALLBACK
        assert result['validated_value'] == 75.0  # Last valid reading
    
    def test_reset(self):
        """Test sensor validator reset"""
        limits = SensorLimits(min_value=0.0, max_value=100.0, name="Test", units="units")
        validator = SensorValidator(limits)
        
        # Trigger failure
        for _ in range(3):
            validator.validate(150.0)
        
        assert validator.status != SensorStatus.HEALTHY
        
        # Reset
        validator.reset()
        assert validator.status == SensorStatus.HEALTHY
        assert validator.consecutive_failures == 0


class TestSystemSensorValidators:
    """Test suite for system sensor validators"""
    
    def test_initialization(self):
        """Test system sensor validators initialization"""
        validators = SystemSensorValidators()
        
        assert validators.pressure is not None
        assert validators.motor_current is not None
        assert validators.valve_position is not None
    
    def test_validate_all_valid(self):
        """Test validation of all valid sensor readings"""
        validators = SystemSensorValidators()
        
        result = validators.validate_all(
            pressure=350.0,
            current=15.0,
            angle=90.0
        )
        
        assert result['all_valid'] is True
        assert result['validated_values']['pressure'] == 350.0
        assert result['validated_values']['motor_current'] == 15.0
        assert result['validated_values']['valve_position'] == 90.0
    
    def test_validate_all_with_fault(self):
        """Test validation with one sensor fault"""
        validators = SystemSensorValidators()
        
        result = validators.validate_all(
            pressure=350.0,
            current=50.0,  # Out of range
            angle=90.0
        )
        
        assert result['all_valid'] is False
        assert result['motor_current']['is_valid'] is False
