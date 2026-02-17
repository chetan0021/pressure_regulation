"""
Unit tests for motor safety monitoring module.
"""

import pytest
from safety.motor_safety import MotorSafetyMonitor, MotorSafetyLimits, MotorFaultType


class TestMotorSafetyMonitor:
    """Test suite for motor safety monitoring"""
    
    def test_initialization(self):
        """Test motor safety monitor initialization"""
        monitor = MotorSafetyMonitor()
        assert monitor.fault_state == MotorFaultType.NONE
        assert monitor.thermal_accumulator == 0.0
    
    def test_normal_operation(self):
        """Test motor safety with normal current"""
        monitor = MotorSafetyMonitor()
        result = monitor.check_safety(current=10.0, time=1.0)
        
        assert result['is_safe'] is True
        assert result['fault_type'] == MotorFaultType.NONE
        assert result['should_shutdown'] is False
    
    def test_overcurrent_detection(self):
        """Test overcurrent fault detection"""
        limits = MotorSafetyLimits(max_current=30.0)
        monitor = MotorSafetyMonitor(limits)
        
        result = monitor.check_safety(current=35.0, time=1.0)
        
        assert result['is_safe'] is False
        assert result['fault_type'] == MotorFaultType.OVERCURRENT
        assert result['should_shutdown'] is True
    
    def test_sensor_fault_detection(self):
        """Test sensor fault detection for out-of-range readings"""
        limits = MotorSafetyLimits(current_sensor_min=-1.0, current_sensor_max=35.0)
        monitor = MotorSafetyMonitor(limits)
        
        result = monitor.check_safety(current=50.0, time=1.0)
        
        assert result['is_safe'] is False
        assert result['fault_type'] == MotorFaultType.SENSOR_FAULT
        assert result['should_shutdown'] is True
    
    def test_stall_detection(self):
        """Test motor stall detection"""
        limits = MotorSafetyLimits(
            stall_current_threshold=25.0,
            stall_duration_threshold=0.5
        )
        monitor = MotorSafetyMonitor(limits)
        
        # High current for short time - should not trigger
        result = monitor.check_safety(current=26.0, time=1.0)
        assert result['is_safe'] is True
        
        # Continue high current - should trigger stall
        result = monitor.check_safety(current=26.0, time=1.6)
        assert result['is_safe'] is False
        assert result['fault_type'] == MotorFaultType.STALL
    
    def test_thermal_protection(self):
        """Test I²t thermal protection"""
        limits = MotorSafetyLimits(
            rated_current=20.0,
            thermal_limit=1.0,
            thermal_time_constant=10.0
        )
        monitor = MotorSafetyMonitor(limits)
        
        # Run at high current to accumulate thermal energy
        for t in range(1, 20):
            result = monitor.check_safety(current=25.0, time=float(t))
            if not result['is_safe']:
                assert result['fault_type'] == MotorFaultType.THERMAL_LIMIT
                break
    
    def test_reset(self):
        """Test safety monitor reset"""
        monitor = MotorSafetyMonitor()
        
        # Trigger fault
        monitor.check_safety(current=35.0, time=1.0)
        assert monitor.fault_state != MotorFaultType.NONE
        
        # Reset
        monitor.reset()
        assert monitor.fault_state == MotorFaultType.NONE
        assert monitor.thermal_accumulator == 0.0
