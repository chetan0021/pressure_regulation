"""
Emergency stop controller for immediate system shutdown.

Provides emergency stop functionality with:
- Immediate motor shutdown
- Valve position hold
- System state preservation
- Recovery procedures
"""

import logging
from typing import Dict, Any
from enum import Enum
from dataclasses import dataclass

logger = logging.getLogger(__name__)


class EmergencyStopState(Enum):
    """Emergency stop system states"""
    NORMAL = "normal"
    STOPPING = "stopping"
    STOPPED = "stopped"
    RECOVERING = "recovering"


@dataclass
class EmergencyStopConfig:
    """Emergency stop configuration"""
    shutdown_timeout: float = 0.5  # seconds (maximum time to complete shutdown)
    recovery_check_interval: float = 1.0  # seconds
    require_manual_reset: bool = True  # Require manual intervention to recover


class EmergencyStopController:
    """
    Emergency stop controller for immediate system shutdown.
    
    Provides fail-safe emergency stop with proper shutdown sequencing
    and recovery procedures.
    """
    
    def __init__(self, config: EmergencyStopConfig = None):
        """
        Initialize emergency stop controller.
        
        Args:
            config: Emergency stop configuration
        """
        self.config = config or EmergencyStopConfig()
        self.state = EmergencyStopState.NORMAL
        self.stop_reason: str = ""
        self.stop_time: float = 0.0
        self.manual_reset_required: bool = False
        
        logger.info("Emergency stop controller initialized")
    
    def trigger_emergency_stop(self, reason: str, time: float) -> Dict[str, Any]:
        """
        Trigger emergency stop.
        
        Args:
            reason: Reason for emergency stop
            time: Current system time
            
        Returns:
            Dictionary with stop commands and status
        """
        if self.state == EmergencyStopState.NORMAL:
            self.state = EmergencyStopState.STOPPING
            self.stop_reason = reason
            self.stop_time = time
            self.manual_reset_required = self.config.require_manual_reset
            
            logger.critical(f"EMERGENCY STOP TRIGGERED: {reason}")
            
            return {
                'motor_voltage': 0.0,  # Cut motor power
                'valve_hold': True,  # Hold valve position
                'system_shutdown': True,
                'state': self.state.value,
                'reason': reason
            }
        else:
            logger.warning(f"Emergency stop already active (state: {self.state.value})")
            return self.get_stop_commands()
    
    def update(self, time: float) -> Dict[str, Any]:
        """
        Update emergency stop state.
        
        Args:
            time: Current system time
            
        Returns:
            Dictionary with current stop commands
        """
        if self.state == EmergencyStopState.STOPPING:
            # Check if shutdown timeout has elapsed
            if time - self.stop_time >= self.config.shutdown_timeout:
                self.state = EmergencyStopState.STOPPED
                logger.info("Emergency stop complete - system halted")
        
        return self.get_stop_commands()
    
    def get_stop_commands(self) -> Dict[str, Any]:
        """Get current emergency stop commands"""
        if self.state in [EmergencyStopState.STOPPING, EmergencyStopState.STOPPED]:
            return {
                'motor_voltage': 0.0,
                'valve_hold': True,
                'system_shutdown': True,
                'state': self.state.value,
                'reason': self.stop_reason
            }
        else:
            return {
                'motor_voltage': None,  # No override
                'valve_hold': False,
                'system_shutdown': False,
                'state': self.state.value,
                'reason': ""
            }
    
    def attempt_recovery(self) -> Dict[str, Any]:
        """
        Attempt to recover from emergency stop.
        
        Returns:
            Dictionary with recovery status
        """
        if self.state != EmergencyStopState.STOPPED:
            return {
                'success': False,
                'message': f"Cannot recover from state: {self.state.value}"
            }
        
        if self.manual_reset_required:
            return {
                'success': False,
                'message': "Manual reset required - call reset() after fault is cleared"
            }
        
        self.state = EmergencyStopState.RECOVERING
        logger.info("Attempting automatic recovery from emergency stop")
        
        return {
            'success': True,
            'message': "Recovery initiated"
        }
    
    def reset(self) -> Dict[str, Any]:
        """
        Manually reset emergency stop (requires operator intervention).
        
        Returns:
            Dictionary with reset status
        """
        if self.state == EmergencyStopState.NORMAL:
            return {
                'success': True,
                'message': "System already in normal state"
            }
        
        logger.warning(f"Manual reset of emergency stop (previous reason: {self.stop_reason})")
        
        self.state = EmergencyStopState.NORMAL
        self.stop_reason = ""
        self.manual_reset_required = False
        
        return {
            'success': True,
            'message': "Emergency stop reset - system ready"
        }
    
    def is_stopped(self) -> bool:
        """Check if system is in emergency stop state"""
        return self.state in [EmergencyStopState.STOPPING, EmergencyStopState.STOPPED]
    
    def get_status(self) -> Dict[str, Any]:
        """Get current emergency stop status"""
        return {
            'state': self.state.value,
            'is_stopped': self.is_stopped(),
            'stop_reason': self.stop_reason,
            'manual_reset_required': self.manual_reset_required
        }
