"""
Industrial Pressure Control System - Main Entry Point

This is the main entry point for the aerospace-grade pressure control system.
It initializes all components and launches the GUI dashboard.

Features:
- High-fidelity physics models (motor, valve, pressure)
- Industrial-grade PID control with anti-windup
- Comprehensive safety monitoring
- Real-time GUI visualization

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

import tkinter as tk
import logging
import logging.config
import numpy as np

# Import system components
from models import DCMotor, RotaryValve, PressureDynamics
from control import PIDController
from simulation import PressureControlSimulator
from gui import PressureControlDashboard
from config import load_parameters
from safety import SafetyManager, MotorSafetyMonitor, PressureSafetyMonitor, ValveSafetyMonitor, EmergencyStopController
from validation import SystemSensorValidators


def setup_logging():
    """Configure logging system."""
    try:
        logging.config.fileConfig('config/logging.conf')
    except:
        # Fallback to basic logging if config file not found
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
    
    logger = logging.getLogger(__name__)
    logger.info("=" * 60)
    logger.info("Industrial Pressure Control System Starting")
    logger.info("=" * 60)
    return logger


def main():
    """
    Main function to initialize and run the pressure control system.
    """
    # Setup logging
    logger = setup_logging()
    
    try:
        # Load system parameters
        logger.info("Loading configuration parameters...")
        params = load_parameters()
        
        # --- Initialize Motor Model ---
        logger.info("Initializing DC motor model...")
        motor = DCMotor(
            V_supply=params['motor']['V_supply'],
            Kt=params['motor']['Kt'],
            Ke=params['motor']['Ke'],
            R=params['motor']['R'],
            L=params['motor']['L'],
            J=params['motor']['J'],
            b=params['motor']['b'],
            gear_ratio=params['motor']['gear_ratio'],
            gear_efficiency=params['motor']['gear_efficiency']
        )
        
        # --- Initialize Valve Model ---
        logger.info("Initializing rotary valve model...")
        valve = RotaryValve(
            mass=params['valve']['mass'],
            radius=params['valve']['radius'],
            b=params['valve']['b'],
            T_static=params['valve']['T_static'],
            T_dynamic=params['valve']['T_dynamic'],
            omega_stribeck=params['valve']['omega_stribeck'],
            min_angle_deg=params['valve']['min_angle_deg'],
            max_angle_deg=params['valve']['max_angle_deg']
        )
        
        # --- Initialize Pressure Model ---
        logger.info("Initializing pressure dynamics model...")
        pressure_model = PressureDynamics(
            beta=params['pressure']['beta'],
            V_base=params['pressure']['V_base'],
            rho=params['pressure']['rho'],
            C_d=params['pressure']['C_d'],
            A_valve_max=params['pressure']['A_valve_max'],
            P_supply=params['pressure']['P_supply'],
            P_tank=params['pressure']['P_tank'],
            tau_first_order=params['pressure'].get('tau_first_order', 0.8)
        )
        
        # --- Initialize PID Controller ---
        logger.info("Initializing PID controller...")
        controller = PIDController(
            Kp=params['pid']['Kp'],
            Ki=params['pid']['Ki'],
            Kd=params['pid']['Kd'],
            setpoint=params['pid']['default_setpoint'],
            u_min=params['pid']['u_min'],
            u_max=params['pid']['u_max'],
            Kt=params['pid']['Kt'],
            tau_d=params['pid']['tau_d'],
            Kff=params['pid']['Kff']
        )
        
        # --- Initialize Safety System (Optional) ---
        logger.info("Initializing safety monitoring system...")
        try:
            motor_safety = MotorSafetyMonitor()
            pressure_safety = PressureSafetyMonitor()
            valve_safety = ValveSafetyMonitor()
            estop = EmergencyStopController()
            
            safety_manager = SafetyManager(
                motor_monitor=motor_safety,
                pressure_monitor=pressure_safety,
                valve_monitor=valve_safety,
                estop=estop
            )
            
            # Initialize sensor validators
            sensor_validators = SystemSensorValidators()
            
            logger.info("Safety systems initialized")
        except Exception as e:
            logger.warning(f"Safety systems not available: {e}")
            safety_manager = None
            sensor_validators = None
        
        # --- Initialize Simulator ---
        logger.info("Initializing simulator...")
        simulator = PressureControlSimulator(
            motor=motor,
            valve=valve,
            pressure_model=pressure_model,
            controller=controller,
            safety_manager=safety_manager,
            sensor_validators=sensor_validators
        )
        
        # Configure disturbance
        if 'simulation' in params:
            disturbance_time = params['simulation'].get('disturbance_time', 4.0)
            disturbance_magnitude = params['simulation'].get('disturbance_magnitude', 50.0)
            simulator.set_disturbance(disturbance_time, disturbance_magnitude)
        
        # --- Create GUI ---
        logger.info("Creating GUI dashboard...")
        root = tk.Tk()
        root.title("Industrial Pressure Control System - Aerospace Grade")
        root.geometry("1400x900")
        
        # Initialize dashboard
        dashboard = PressureControlDashboard(root, simulator)
        
        logger.info("=" * 60)
        logger.info("System initialized successfully")
        logger.info("Ready to start simulation")
        logger.info("=" * 60)
        
        # Start GUI event loop
        root.mainloop()
        
    except Exception as e:
        logger.error(f"Fatal error during initialization: {e}", exc_info=True)
        raise


if __name__ == "__main__":
    main()
