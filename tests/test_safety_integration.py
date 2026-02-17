"""
Integration tests for safety manager coordinating all safety systems.
"""

import pytest
from safety.safety_manager import SafetyManager, SystemState


class TestSafetyManager:
    """Test suite for safety manager integration"""
    
    def test_initialization(self):
        """Test safety manager initialization"""
        manager = SafetyManager()
        
        assert manager.motor_monitor is not None
        assert manager.pressure_monitor is not None
        assert manager.valve_monitor is not None
        assert manager.estop is not None
    
    def test_normal_operation(self):
        """Test safety manager with all systems normal"""
        manager = SafetyManager()
        
        state = SystemState(
            motor_current=15.0,
            pressure=350.0,
            valve_angle=90.0,
            time=1.0
        )
        
        result = manager.check_all_safety(state)
        
        assert result['is_safe'] is True
        assert result['should_shutdown'] is False
        assert len(result['fault_messages']) == 0
    
    def test_motor_fault_triggers_estop(self):
        """Test that motor fault triggers emergency stop"""
        manager = SafetyManager()
        
        state = SystemState(
            motor_current=35.0,  # Overcurrent
            pressure=350.0,
            valve_angle=90.0,
            time=1.0
        )
        
        result = manager.check_all_safety(state)
        
        assert result['is_safe'] is False
        assert result['should_shutdown'] is True
        assert result['motor_status']['fault_type'].value == 'overcurrent'
        assert manager.estop.is_stopped() is True
    
    def test_pressure_fault_triggers_estop(self):
        """Test that pressure fault triggers emergency stop"""
        manager = SafetyManager()
        
        state = SystemState(
            motor_current=15.0,
            pressure=750.0,  # Overpressure
            valve_angle=90.0,
            time=1.0
        )
        
        result = manager.check_all_safety(state)
        
        assert result['is_safe'] is False
        assert result['should_shutdown'] is True
        assert result['pressure_status']['fault_type'].value == 'overpressure'
    
    def test_valve_fault_triggers_estop(self):
        """Test that valve fault triggers emergency stop"""
        manager = SafetyManager()
        
        state = SystemState(
            motor_current=15.0,
            pressure=350.0,
            valve_angle=200.0,  # Out of range
            time=1.0
        )
        
        result = manager.check_all_safety(state)
        
        assert result['is_safe'] is False
        assert result['should_shutdown'] is True
        assert result['valve_status']['fault_type'].value == 'position_limit'
    
    def test_estop_persists_across_checks(self):
        """Test that emergency stop persists until reset"""
        manager = SafetyManager()
        
        # Trigger fault
        state1 = SystemState(
            motor_current=35.0,
            pressure=350.0,
            valve_angle=90.0,
            time=1.0
        )
        manager.check_all_safety(state1)
        
        # Check with normal values - should still be stopped
        state2 = SystemState(
            motor_current=15.0,
            pressure=350.0,
            valve_angle=90.0,
            time=2.0
        )
        result = manager.check_all_safety(state2)
        
        assert result['is_safe'] is False
        assert manager.estop.is_stopped() is True
    
    def test_reset_all(self):
        """Test resetting all safety systems"""
        manager = SafetyManager()
        
        # Trigger fault
        state = SystemState(
            motor_current=35.0,
            pressure=350.0,
            valve_angle=90.0,
            time=1.0
        )
        manager.check_all_safety(state)
        
        assert manager.estop.is_stopped() is True
        
        # Reset
        manager.reset_all()
        
        assert manager.estop.is_stopped() is False
        assert manager.fault_count == 0
