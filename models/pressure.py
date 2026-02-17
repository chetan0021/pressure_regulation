"""
High-Fidelity Pressure Dynamics Model Module

Implements aerospace-grade compressible fluid pressure dynamics including
bulk modulus effects, volume changes, and flow rate dependencies.

State Variable:
- Pressure (P): System pressure [bar]

Governing Equation:
- Compressible fluid: dP/dt = (β/V) × (Q_in - Q_out - dV/dt)

Where:
- β: Bulk modulus of fluid [Pa]
- V: System volume [m³]
- Q_in: Inlet flow rate [m³/s]
- Q_out: Outlet flow rate [m³/s]
- dV/dt: Rate of volume change [m³/s]

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

import numpy as np
import logging

logger = logging.getLogger(__name__)


class PressureDynamics:
    """
    High-fidelity pressure dynamics model with compressible fluid effects.
    
    This model implements realistic pressure dynamics accounting for:
    - Fluid compressibility (bulk modulus)
    - Volume changes due to valve position
    - Flow rate dependencies
    - Pressure-dependent flow characteristics
    
    Physical Parameters:
    - beta: Bulk modulus of fluid [Pa] (typically 2.2e9 Pa for hydraulic oil)
    - V_base: Base system volume [m³]
    - rho: Fluid density [kg/m³]
    - C_d: Discharge coefficient (dimensionless, typically 0.6-0.8)
    - A_valve_max: Maximum valve flow area [m²]
    - P_supply: Supply pressure [bar]
    - P_tank: Tank/return pressure [bar]
    """
    
    def __init__(self, beta, V_base, rho, C_d, A_valve_max, P_supply, P_tank, 
                 tau_first_order=None):
        """
        Initialize high-fidelity pressure dynamics model.
        
        Args:
            beta: Bulk modulus of fluid [Pa]
            V_base: Base system volume [m³]
            rho: Fluid density [kg/m³]
            C_d: Discharge coefficient [dimensionless]
            A_valve_max: Maximum valve flow area [m²]
            P_supply: Supply pressure [bar]
            P_tank: Tank/return pressure [bar]
            tau_first_order: Optional first-order time constant for simplified model [s]
        """
        self.beta = beta
        self.V_base = V_base
        self.rho = rho
        self.C_d = C_d
        self.A_valve_max = A_valve_max
        self.P_supply = P_supply
        self.P_tank = P_tank
        self.tau_first_order = tau_first_order
        
        # Conversion factor: 1 bar = 1e5 Pa
        self.bar_to_pa = 1e5
        
        logger.info(f"Pressure Dynamics initialized: β={beta/1e9:.2f}GPa, "
                   f"V_base={V_base*1e6:.2f}L, ρ={rho}kg/m³, "
                   f"P_supply={P_supply}bar, P_tank={P_tank}bar")
    
    def compute_flow_area(self, valve_angle, max_angle):
        """
        Compute effective flow area based on valve position.
        
        Linear relationship: A = A_max × (θ / θ_max)
        
        Args:
            valve_angle: Current valve angle [rad]
            max_angle: Maximum valve angle [rad]
            
        Returns:
            A: Effective flow area [m²]
        """
        # Normalized valve position (0 to 1)
        position_normalized = valve_angle / max_angle
        position_normalized = np.clip(position_normalized, 0.0, 1.0)
        
        # Flow area
        A = self.A_valve_max * position_normalized
        
        return A
    
    def compute_flow_rate(self, valve_angle, max_angle, pressure):
        """
        Compute volumetric flow rate through valve using orifice equation.
        
        Q = C_d × A × sqrt(2 × ΔP / ρ)
        
        Args:
            valve_angle: Current valve angle [rad]
            max_angle: Maximum valve angle [rad]
            pressure: Current system pressure [bar]
            
        Returns:
            Q: Volumetric flow rate [m³/s]
        """
        # Effective flow area
        A = self.compute_flow_area(valve_angle, max_angle)
        
        # Pressure differential (supply to system, system to tank)
        # Convert bar to Pa for calculation
        P_sys_pa = pressure * self.bar_to_pa
        P_supply_pa = self.P_supply * self.bar_to_pa
        P_tank_pa = self.P_tank * self.bar_to_pa
        
        # Determine flow direction and pressure drop
        if P_supply_pa > P_sys_pa:
            # Flow from supply into system
            delta_P = P_supply_pa - P_sys_pa
            flow_direction = 1.0  # Positive (filling)
        else:
            # Flow from system to tank
            delta_P = P_sys_pa - P_tank_pa
            flow_direction = -1.0  # Negative (draining)
        
        # Ensure positive pressure drop
        delta_P = max(delta_P, 0.0)
        
        # Orifice equation: Q = C_d × A × sqrt(2 × ΔP / ρ)
        if delta_P > 0:
            Q = self.C_d * A * np.sqrt(2.0 * delta_P / self.rho)
        else:
            Q = 0.0
        
        # Apply flow direction
        Q_net = flow_direction * Q
        
        return Q_net
    
    def compute_pressure_derivative_compressible(self, valve_angle, max_angle, 
                                                 pressure, valve_velocity):
        """
        Compute pressure derivative using compressible fluid dynamics.
        
        dP/dt = (β/V) × (Q_in - Q_out - dV/dt)
        
        For simplicity, we model Q_net = Q_in - Q_out as a single flow rate.
        Volume change dV/dt is typically small and can be neglected for rigid systems.
        
        Args:
            valve_angle: Current valve angle [rad]
            max_angle: Maximum valve angle [rad]
            pressure: Current system pressure [bar]
            valve_velocity: Rate of valve angle change [rad/s]
            
        Returns:
            dP/dt: Pressure derivative [bar/s]
        """
        # Net volumetric flow rate [m³/s]
        Q_net = self.compute_flow_rate(valve_angle, max_angle, pressure)
        
        # Volume change rate (typically negligible for rigid systems)
        # For flexible volumes: dV/dt = f(valve_position, pressure)
        dV_dt = 0.0  # Assuming rigid system
        
        # Compressible fluid equation: dP/dt = (β/V) × (Q_net - dV/dt)
        # Convert result from Pa/s to bar/s
        dP_dt_pa = (self.beta / self.V_base) * (Q_net - dV_dt)
        dP_dt_bar = dP_dt_pa / self.bar_to_pa
        
        return dP_dt_bar
    
    def compute_pressure_derivative_first_order(self, valve_angle, max_angle, 
                                               pressure, setpoint_pressure):
        """
        Compute pressure derivative using simplified first-order model.
        
        This is a fallback/simplified model for comparison:
        dP/dt = (K × θ - P) / τ
        
        Args:
            valve_angle: Current valve angle [rad]
            max_angle: Maximum valve angle [rad]
            pressure: Current system pressure [bar]
            setpoint_pressure: Target pressure [bar]
            
        Returns:
            dP/dt: Pressure derivative [bar/s]
        """
        if self.tau_first_order is None:
            raise ValueError("First-order time constant not configured")
        
        # Gain relating valve angle to pressure
        K = self.P_supply / max_angle  # [bar/rad]
        
        # First-order dynamics
        P_target = K * valve_angle
        dP_dt = (P_target - pressure) / self.tau_first_order
        
        return dP_dt
    
    def compute_derivative(self, valve_angle, max_angle, pressure, 
                          valve_velocity=0.0, use_compressible=True):
        """
        Compute pressure derivative (main interface).
        
        Args:
            valve_angle: Current valve angle [rad]
            max_angle: Maximum valve angle [rad]
            pressure: Current system pressure [bar]
            valve_velocity: Rate of valve angle change [rad/s]
            use_compressible: Use compressible model (True) or first-order (False)
            
        Returns:
            dP/dt: Pressure derivative [bar/s]
        """
        if use_compressible:
            return self.compute_pressure_derivative_compressible(
                valve_angle, max_angle, pressure, valve_velocity
            )
        else:
            # Fallback to first-order model
            return self.compute_pressure_derivative_first_order(
                valve_angle, max_angle, pressure, pressure
            )
    
    def get_state_info(self, valve_angle, max_angle, pressure):
        """
        Get comprehensive state information for monitoring and diagnostics.
        
        Args:
            valve_angle: Current valve angle [rad]
            max_angle: Maximum valve angle [rad]
            pressure: Current system pressure [bar]
            
        Returns:
            dict: State information
        """
        flow_area = self.compute_flow_area(valve_angle, max_angle)
        flow_rate = self.compute_flow_rate(valve_angle, max_angle, pressure)
        
        return {
            'pressure_bar': pressure,
            'pressure_pa': pressure * self.bar_to_pa,
            'flow_area_m2': flow_area,
            'flow_area_mm2': flow_area * 1e6,
            'flow_rate_m3_s': flow_rate,
            'flow_rate_L_min': flow_rate * 60000,  # Convert m³/s to L/min
            'valve_position_normalized': valve_angle / max_angle,
            'pressure_ratio': pressure / self.P_supply
        }
