"""
Test configuration and fixtures for pressure regulation system tests.
"""

import pytest
import logging
from typing import Dict, Any

# Configure logging for tests
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


@pytest.fixture
def motor_test_params() -> Dict[str, float]:
    """Standard motor parameters for testing"""
    return {
        'voltage': 36.0,
        'resistance': 1.2,
        'inductance': 0.05,
        'Kt': 0.8,
        'Ke': 0.8,
        'max_current': 30.0,
        'rated_current': 20.0
    }


@pytest.fixture
def valve_test_params() -> Dict[str, float]:
    """Standard valve parameters for testing"""
    return {
        'mass': 100.0,
        'radius': 0.35,
        'inertia': 6.125,
        'friction': 120.0,
        'min_angle': 0.0,
        'max_angle': 180.0
    }


@pytest.fixture
def pressure_test_params() -> Dict[str, float]:
    """Standard pressure parameters for testing"""
    return {
        'time_constant': 0.8,
        'max_pressure': 700.0,
        'gain': 3.89,
        'min_pressure': 0.0
    }


@pytest.fixture
def pid_test_params() -> Dict[str, float]:
    """Standard PID parameters for testing"""
    return {
        'Kp': 0.5,
        'Ki': 0.1,
        'Kd': 0.05,
        'setpoint': 350.0
    }


@pytest.fixture
def safety_test_limits() -> Dict[str, Any]:
    """Safety limits for testing"""
    return {
        'motor': {
            'max_current': 30.0,
            'rated_current': 20.0,
            'stall_current_threshold': 25.0,
            'stall_duration_threshold': 0.5
        },
        'pressure': {
            'max_pressure': 700.0,
            'safe_max_pressure': 650.0,
            'min_pressure': 0.0,
            'max_rate_of_change': 200.0
        },
        'valve': {
            'min_angle': 0.0,
            'max_angle': 180.0,
            'max_velocity': 90.0
        }
    }
