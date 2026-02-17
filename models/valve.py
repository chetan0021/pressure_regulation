"""
High-Fidelity Rotary Valve Model Module

Implements aerospace-grade rotary valve dynamics with second-order servo behavior,
including inertia, damping, and comprehensive friction modeling.

State Variables:
- Angle (θ): Valve angular position [rad]
- Angular velocity (ω): Valve rotation rate [rad/s]

Governing Equations:
- Kinematics: dθ/dt = ω
- Dynamics: J·dω/dt = T_motor - T_friction - T_pressure

Friction Model:
- Static friction (Coulomb)
- Dynamic friction (viscous)
- Stribeck effect (transition between static and dynamic)

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

import numpy as np
import logging

logger = logging.getLogger(__name__)


class RotaryValve:
    """
    High-fidelity rotary valve model with second-order servo dynamics.
    
    This model implements a realistic valve with inertial effects, comprehensive
    friction modeling, and pressure-induced torques suitable for aerospace simulation.
    
    Physical Parameters:
    - mass: Valve disk mass [kg]
    - radius: Valve disk radius [m]
    - J: Moment of inertia [kg·m²] (calculated from mass and radius)
    - b: Viscous damping coefficient [Nm·s/rad]
    - T_static: Static (Coulomb) friction torque [Nm]
    - T_dynamic: Dynamic (viscous) friction coefficient [Nm·s/rad]
    - omega_stribeck: Stribeck velocity [rad/s]
    - min_angle: Minimum valve angle [rad]
    - max_angle: Maximum valve angle [rad]
    """
    
    def __init__(self, mass, radius, b, T_static, T_dynamic, omega_stribeck, 
                 min_angle_deg=0.0, max_angle_deg=180.0):
        """
        Initialize high-fidelity rotary valve model.
        
        Args:
            mass: Valve disk mass [kg]
            radius: Valve disk radius [m]
            b: Viscous damping coefficient [Nm·s/rad]
            T_static: Static friction torque [Nm]
            T_dynamic: Dynamic friction coefficient [Nm·s/rad]
            omega_stribeck: Stribeck velocity [rad/s]
            min_angle_deg: Minimum valve angle [degrees]
            max_angle_deg: Maximum valve angle [degrees]
        """
        self.mass = mass
        self.radius = radius
        self.b = b
        self.T_static = T_static
        self.T_dynamic = T_dynamic
        self.omega_stribeck = omega_stribeck
        
        # Calculate moment of inertia for solid disk: J = (1/2)·m·r²
        self.J = 0.5 * mass * radius**2
        
        # Angle limits (convert to radians)
        self.min_angle = np.deg2rad(min_angle_deg)
        self.max_angle = np.deg2rad(max_angle_deg)
        
        logger.info(f"Rotary Valve initialized: mass={mass}kg, radius={radius}m, "
                   f"J={self.J:.3f}kg·m², b={b}Nm·s/rad, T_static={T_static}Nm, "
                   f"angle_range=[{min_angle_deg}°, {max_angle_deg}°]")
    
    def compute_friction_torque(self, omega):
        """
        Compute friction torque using advanced Stribeck friction model.
        
        The Stribeck model captures:
        - Static friction at zero velocity
        - Stribeck effect (friction decrease at low velocities)
        - Viscous friction at higher velocities
        
        T_friction = sign(ω) × [T_static × exp(-(ω/ω_s)²) + T_dynamic × |ω|]
        
        Args:
            omega: Angular velocity [rad/s]
            
        Returns:
            T_friction: Friction torque [Nm]
        """
        if abs(omega) < 1e-6:
            # At rest: static friction (will be limited by applied torque)
            return 0.0  # Actual static friction is handled by stiction logic
        
        # Sign of velocity
        sign_omega = np.sign(omega)
        
        # Stribeck effect: friction decreases exponentially from static to dynamic
        # as velocity increases from zero
        stribeck_term = self.T_static * np.exp(-(omega / self.omega_stribeck)**2)
        
        # Viscous friction term (proportional to velocity)
        viscous_term = self.T_dynamic * abs(omega)
        
        # Total friction torque
        T_friction = sign_omega * (stribeck_term + viscous_term)
        
        return T_friction
    
    def compute_pressure_torque(self, pressure, angle):
        """
        Compute torque induced by pressure acting on valve.
        
        Simplified model: pressure creates a restoring torque that opposes
        valve opening. This represents the force of pressurized fluid on the valve.
        
        T_pressure = K_p × P × sin(θ)
        
        Args:
            pressure: System pressure [bar]
            angle: Current valve angle [rad]
            
        Returns:
            T_pressure: Pressure-induced torque [Nm]
        """
        # Pressure torque coefficient (empirical, based on valve geometry)
        # For a 0.35m radius valve with pressure acting on area
        K_p = 0.01  # [Nm/bar] - tunable parameter
        
        # Torque opposes opening (maximum at 90°)
        T_pressure = K_p * pressure * np.sin(angle)
        
        return T_pressure
    
    def compute_angle_derivative(self, omega):
        """
        Compute time derivative of valve angle (kinematics).
        
        dθ/dt = ω
        
        Args:
            omega: Angular velocity [rad/s]
            
        Returns:
            dθ/dt: Angular velocity [rad/s]
        """
        return omega
    
    def compute_velocity_derivative(self, T_motor, omega, pressure, angle):
        """
        Compute time derivative of angular velocity (dynamics).
        
        J·dω/dt = T_motor - T_friction - T_damping - T_pressure
        
        Args:
            T_motor: Motor torque applied to valve [Nm]
            omega: Current angular velocity [rad/s]
            pressure: System pressure [bar]
            angle: Current valve angle [rad]
            
        Returns:
            dω/dt: Angular acceleration [rad/s²]
        """
        # Friction torque (Stribeck model)
        T_friction = self.compute_friction_torque(omega)
        
        # Viscous damping torque
        T_damping = self.b * omega
        
        # Pressure-induced torque
        T_pressure = self.compute_pressure_torque(pressure, angle)
        
        # Newton's second law for rotation
        # J·dω/dt = T_motor - T_friction - T_damping - T_pressure
        dω_dt = (T_motor - T_friction - T_damping - T_pressure) / self.J
        
        return dω_dt
    
    def compute_derivatives(self, T_motor, angle, omega, pressure):
        """
        Compute both kinematic and dynamic derivatives simultaneously.
        
        Args:
            T_motor: Motor torque [Nm]
            angle: Valve angle [rad]
            omega: Angular velocity [rad/s]
            pressure: System pressure [bar]
            
        Returns:
            tuple: (dθ/dt, dω/dt)
        """
        dθ_dt = self.compute_angle_derivative(omega)
        dω_dt = self.compute_velocity_derivative(T_motor, omega, pressure, angle)
        
        return dθ_dt, dω_dt
    
    def check_angle_limits(self, angle):
        """
        Check if angle is within physical limits.
        
        Args:
            angle: Valve angle [rad]
            
        Returns:
            tuple: (is_within_limits, clamped_angle)
        """
        is_within = self.min_angle <= angle <= self.max_angle
        clamped = np.clip(angle, self.min_angle, self.max_angle)
        return is_within, clamped
    
    def apply_hard_stops(self, angle, omega):
        """
        Apply hard stop behavior at angle limits.
        
        When valve hits physical stops, velocity is set to zero.
        
        Args:
            angle: Current angle [rad]
            omega: Current angular velocity [rad/s]
            
        Returns:
            tuple: (corrected_angle, corrected_omega)
        """
        if angle <= self.min_angle and omega < 0:
            # Hit minimum stop while moving negative
            return self.min_angle, 0.0
        elif angle >= self.max_angle and omega > 0:
            # Hit maximum stop while moving positive
            return self.max_angle, 0.0
        else:
            return angle, omega
    
    def get_flow_coefficient(self, angle):
        """
        Get flow coefficient based on valve angle.
        
        Cv = Cv_max × (θ / θ_max)
        
        Args:
            angle: Valve angle [rad]
            
        Returns:
            Cv: Flow coefficient [0-1]
        """
        # Normalized angle (0 to 1)
        angle_normalized = (angle - self.min_angle) / (self.max_angle - self.min_angle)
        
        # Flow coefficient (linear relationship for simplicity)
        # Can be replaced with more complex valve characteristics
        Cv = np.clip(angle_normalized, 0.0, 1.0)
        
        return Cv
    
    def get_state_info(self, angle, omega, T_motor, pressure):
        """
        Get comprehensive state information for monitoring and diagnostics.
        
        Args:
            angle: Valve angle [rad]
            omega: Angular velocity [rad/s]
            T_motor: Applied motor torque [Nm]
            pressure: System pressure [bar]
            
        Returns:
            dict: State information
        """
        return {
            'angle_rad': angle,
            'angle_deg': np.rad2deg(angle),
            'omega': omega,
            'T_motor': T_motor,
            'T_friction': self.compute_friction_torque(omega),
            'T_damping': self.b * omega,
            'T_pressure': self.compute_pressure_torque(pressure, angle),
            'flow_coefficient': self.get_flow_coefficient(angle),
            'at_min_limit': angle <= self.min_angle,
            'at_max_limit': angle >= self.max_angle
        }
