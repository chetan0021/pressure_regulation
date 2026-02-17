"""
High-Fidelity DC Motor Model Module

Implements aerospace-grade DC motor dynamics with full state-space representation
including electrical (armature circuit) and mechanical (rotor) subsystems.

State Variables:
- Current (I): Armature current [A]
- Angular velocity (ω): Motor shaft speed [rad/s]

Governing Equations:
- Electrical: L·dI/dt = V - R·I - Ke·ω
- Mechanical: J·dω/dt = Kt·I - b·ω - T_load

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

import numpy as np
import logging

logger = logging.getLogger(__name__)


class DCMotor:
    """
    High-fidelity DC motor model with coupled electrical and mechanical dynamics.
    
    This model implements the full state-space representation of a permanent magnet
    DC motor suitable for aerospace-grade simulation.
    
    Physical Parameters:
    - V_supply: Nominal supply voltage [V]
    - R: Armature resistance [Ω]
    - L: Armature inductance [H]
    - Kt: Torque constant [Nm/A]
    - Ke: Back-EMF constant [V/(rad/s)]
    - J: Rotor inertia [kg·m²]
    - b: Viscous damping coefficient [Nm·s/rad]
    - gear_ratio: Gearbox reduction ratio (motor:load)
    - gear_efficiency: Gearbox efficiency [0-1]
    """
    
    def __init__(self, V_supply, Kt, Ke, R, L, J, b, gear_ratio, gear_efficiency):
        """
        Initialize high-fidelity DC motor model.
        
        Args:
            V_supply: Nominal supply voltage [V]
            Kt: Torque constant [Nm/A]
            Ke: Back-EMF constant [V/(rad/s)]
            R: Armature resistance [Ω]
            L: Armature inductance [H]
            J: Rotor inertia [kg·m²]
            b: Viscous damping coefficient [Nm·s/rad]
            gear_ratio: Gearbox ratio (motor:load)
            gear_efficiency: Gearbox efficiency [0-1]
        """
        self.V_supply = V_supply
        self.Kt = Kt
        self.Ke = Ke
        self.R = R
        self.L = L
        self.J = J
        self.b = b
        self.gear_ratio = gear_ratio
        self.gear_efficiency = gear_efficiency
        
        # Derived parameters
        self.electrical_time_constant = L / R  # τ_e = L/R
        self.mechanical_time_constant = J / b  # τ_m = J/b
        
        logger.info(f"DC Motor initialized: V={V_supply}V, Kt={Kt}Nm/A, Ke={Ke}V/(rad/s), "
                   f"R={R}Ω, L={L}H, J={J}kg·m², b={b}Nm·s/rad, "
                   f"τ_e={self.electrical_time_constant:.4f}s, τ_m={self.mechanical_time_constant:.4f}s")
    
    def compute_current_derivative(self, V_input, omega, current):
        """
        Compute the time derivative of armature current.
        
        Electrical equation: L·dI/dt = V_input - R·I - Ke·ω
        
        Args:
            V_input: Applied voltage [V]
            omega: Motor angular velocity [rad/s]
            current: Current armature current [A]
            
        Returns:
            dI/dt: Current derivative [A/s]
        """
        # Clamp input voltage to supply limits
        V_clamped = np.clip(V_input, -self.V_supply, self.V_supply)
        
        # Back-EMF voltage
        V_bemf = self.Ke * omega
        
        # Voltage across resistance
        V_resistive = self.R * current
        
        # Kirchhoff's voltage law: V = L·dI/dt + R·I + Ke·ω
        # Rearranged: dI/dt = (V - R·I - Ke·ω) / L
        dI_dt = (V_clamped - V_resistive - V_bemf) / self.L
        
        return dI_dt
    
    def compute_speed_derivative(self, current, T_load):
        """
        Compute the time derivative of motor angular velocity.
        
        Mechanical equation: J·dω/dt = Kt·I - b·ω - T_load
        
        Args:
            current: Armature current [A]
            T_load: Load torque at motor shaft (before gearbox) [Nm]
            
        Returns:
            dω/dt: Angular acceleration [rad/s²]
        """
        # Electromagnetic torque
        T_motor = self.Kt * current
        
        # Viscous damping torque
        T_damping = self.b * self.get_omega_from_state()  # Will be passed as state
        
        # Newton's second law for rotation: J·dω/dt = T_motor - T_damping - T_load
        dω_dt = (T_motor - T_damping - T_load) / self.J
        
        return dω_dt
    
    def compute_derivatives(self, V_input, current, omega, T_load_at_motor):
        """
        Compute both electrical and mechanical derivatives simultaneously.
        
        Args:
            V_input: Applied voltage [V]
            current: Armature current [A]
            omega: Motor angular velocity [rad/s]
            T_load_at_motor: Load torque reflected to motor shaft [Nm]
            
        Returns:
            tuple: (dI/dt, dω/dt)
        """
        dI_dt = self.compute_current_derivative(V_input, omega, current)
        
        # Mechanical dynamics with damping
        T_motor = self.Kt * current
        T_damping = self.b * omega
        dω_dt = (T_motor - T_damping - T_load_at_motor) / self.J
        
        return dI_dt, dω_dt
    
    def reflect_load_torque_to_motor(self, T_load_at_valve):
        """
        Reflect load torque from valve (through gearbox) to motor shaft.
        
        Args:
            T_load_at_valve: Torque at valve/load side [Nm]
            
        Returns:
            T_load_at_motor: Equivalent torque at motor shaft [Nm]
        """
        # Torque reflection through gearbox:
        # T_motor = T_load / (N × η) where N is gear ratio, η is efficiency
        T_load_at_motor = T_load_at_valve / (self.gear_ratio * self.gear_efficiency)
        return T_load_at_motor
    
    def get_output_torque(self, current):
        """
        Get output torque at valve side (after gearbox).
        
        Args:
            current: Motor current [A]
            
        Returns:
            T_output: Torque at valve/load side [Nm]
        """
        # Motor torque
        T_motor = self.Kt * current
        
        # Amplified by gear ratio, reduced by efficiency
        T_output = T_motor * self.gear_ratio * self.gear_efficiency
        
        return T_output
    
    def get_output_velocity(self, omega_motor):
        """
        Get output angular velocity at valve side (after gearbox).
        
        Args:
            omega_motor: Motor shaft angular velocity [rad/s]
            
        Returns:
            omega_output: Angular velocity at valve/load side [rad/s]
        """
        # Velocity reduced by gear ratio
        omega_output = omega_motor / self.gear_ratio
        return omega_output
    
    def get_state_info(self, current, omega):
        """
        Get comprehensive state information for monitoring and diagnostics.
        
        Args:
            current: Armature current [A]
            omega: Motor angular velocity [rad/s]
            
        Returns:
            dict: State information
        """
        return {
            'current': current,
            'omega_motor': omega,
            'omega_output': self.get_output_velocity(omega),
            'T_motor': self.Kt * current,
            'T_output': self.get_output_torque(current),
            'V_bemf': self.Ke * omega,
            'power_electrical': current * self.V_supply,
            'power_mechanical': self.Kt * current * omega
        }
