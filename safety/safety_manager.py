"""
Safety manager to coordinate all safety monitoring modules.

Centralizes safety monitoring and provides unified fault detection
and emergency response coordination.
"""

import logging
from typing import Dict, Any, Optional
from dataclasses import dataclass

from .motor_safety import MotorSafetyMonitor, MotorSafetyLimits
from .pressure_safety import PressureSafetyMonitor, PressureSafetyLimits
from .valve_safety import ValveSafetyMonitor, ValveSafetyLimits
from .emergency_stop import EmergencyStopController, EmergencyStopConfig

logger = logging.getLogger(__name__)


@dataclass
class SystemState:
    """Current system state for safety monitoring"""
    motor_current: float
    pressure: float
    valve_angle: float
    time: float
    commanded_valve_velocity: Optional[float] = None


class SafetyManager:
    """
    Centralized safety management system.
    
    Coordinates all safety monitors and provides unified fault detection,
    emergency response, and system protection.
    """
    
    def __init__(self,
                 motor_limits: Optional[MotorSafetyLimits] = None,
                 pressure_limits: Optional[PressureSafetyLimits] = None,
                 valve_limits: Optional[ValveSafetyLimits] = None,
                 estop_config: Optional[EmergencyStopConfig] = None):
    def __init__(self, motor_monitor=None, pressure_monitor=None, 
                 valve_monitor=None, estop=None):
        """
        Initialize safety manager with all monitoring subsystems.
        
        Args:
            motor_monitor: MotorSafetyMonitor instance
            pressure_monitor: PressureSafetyMonitor instance
            valve_monitor: ValveSafetyMonitor instance
            estop: EmergencyStopController instance
        """
        self.motor_monitor = motor_monitor or MotorSafetyMonitor()
        self.pressure_monitor = pressure_monitor or PressureSafetyMonitor()
        self.valve_monitor = valve_monitor or ValveSafetyMonitor()
        self.estop = estop or EmergencyStopController()
        
        self.fault_count: int = 0
        self.last_fault_message: str = ""
        
        logger.info("Safety manager initialized with all monitoring subsystems")
    
    def check_all_safety(self, state: SystemState) -> Dict[str, Any]:
        """
        Check all safety monitors.
        
        Args:
            state: Current system state
            
        Returns:
            Dictionary containing:
                - is_safe: bool (overall safety status)
                - motor_status: dict
                - pressure_status: dict
                - valve_status: dict
                - estop_status: dict
                - should_shutdown: bool
                - fault_messages: list of str
        """
        # If already in emergency stop, maintain stop state
        if self.estop.is_stopped():
            estop_commands = self.estop.update(state.time)
            return {
                'is_safe': False,
                'motor_status': {'is_safe': False},
                'pressure_status': {'is_safe': False},
                'valve_status': {'is_safe': False},
                'estop_status': estop_commands,
                'should_shutdown': True,
                'fault_messages': [f"Emergency stop active: {estop_commands['reason']}"]
            }
        
        # Check all safety monitors
        motor_status = self.motor_monitor.check_safety(state.motor_current, state.time)
        pressure_status = self.pressure_monitor.check_safety(state.pressure, state.time)
        valve_status = self.valve_monitor.check_safety(
            state.valve_angle, 
            state.time,
            state.commanded_valve_velocity
        )
        
        # Collect fault messages
        fault_messages = []
        if not motor_status['is_safe']:
            fault_messages.append(f"Motor: {motor_status['message']}")
        if not pressure_status['is_safe']:
            fault_messages.append(f"Pressure: {pressure_status['message']}")
        if not valve_status['is_safe']:
            fault_messages.append(f"Valve: {valve_status['message']}")
        
        # Determine if emergency stop is needed
        should_shutdown = (
            motor_status['should_shutdown'] or
            pressure_status['should_shutdown'] or
            valve_status['should_shutdown']
        )
        
        # Trigger emergency stop if needed
        estop_status = {}
        if should_shutdown:
            self.fault_count += 1
            fault_summary = "; ".join(fault_messages)
            self.last_fault_message = fault_summary
            estop_status = self.estop.trigger_emergency_stop(fault_summary, state.time)
            logger.critical(f"Safety violation detected - triggering emergency stop: {fault_summary}")
        else:
            estop_status = self.estop.get_stop_commands()
        
        # Overall safety status
        is_safe = (
            motor_status['is_safe'] and
            pressure_status['is_safe'] and
            valve_status['is_safe'] and
            not self.estop.is_stopped()
        )
        
        return {
            'is_safe': is_safe,
            'motor_status': motor_status,
            'pressure_status': pressure_status,
            'valve_status': valve_status,
            'estop_status': estop_status,
            'should_shutdown': should_shutdown,
            'fault_messages': fault_messages
        }
    
    def reset_all(self):
        """Reset all safety monitors and emergency stop"""
        logger.warning("Resetting all safety systems - operator intervention")
        self.motor_monitor.reset()
        self.pressure_monitor.reset()
        self.valve_monitor.reset()
        self.estop.reset()
        self.fault_count = 0
        self.last_fault_message = ""
    
    def get_comprehensive_status(self) -> Dict[str, Any]:
        """Get comprehensive status of all safety systems"""
        return {
            'motor': self.motor_monitor.get_status(),
            'pressure': self.pressure_monitor.get_status(),
            'valve': self.valve_monitor.get_status(),
            'estop': self.estop.get_status(),
            'fault_count': self.fault_count,
            'last_fault': self.last_fault_message
        }
