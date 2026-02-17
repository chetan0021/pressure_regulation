"""
Industrial-Grade PID Controller Module

Implements a robust PID controller with advanced features for aerospace/industrial applications:
- Anti-windup (back-calculation method)
- Derivative filtering (first-order low-pass)
- Output saturation handling
- Feedforward compensation
- Bumpless transfer for setpoint changes

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

import numpy as np
import logging

logger = logging.getLogger(__name__)


class PIDController:
    """
    Industrial-grade PID controller with anti-windup and derivative filtering.
    
    Control Law:
    u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·de/dt + u_ff
    
    Features:
    - Anti-windup: Back-calculation method to prevent integral windup
    - Derivative filtering: First-order low-pass filter on derivative term
    - Output saturation: Configurable min/max limits
    - Feedforward: Model-based compensation
    - Bumpless transfer: Smooth setpoint changes
    
    Parameters:
    - Kp: Proportional gain
    - Ki: Integral gain
    - Kd: Derivative gain
    - setpoint: Desired pressure setpoint [bar]
    - u_min, u_max: Output saturation limits [V]
    - Kt: Anti-windup gain (back-calculation)
    - tau_d: Derivative filter time constant [s]
    - Kff: Feedforward gain
    """
    
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, 
                 u_min=-36.0, u_max=36.0,
                 Kt=None, tau_d=0.01, Kff=0.0):
        """
        Initialize industrial-grade PID controller.
        
        Args:
            Kp: Proportional gain
            Ki: Integral gain
            Kd: Derivative gain
            setpoint: Initial setpoint [bar]
            u_min: Minimum output [V]
            u_max: Maximum output [V]
            Kt: Anti-windup gain (if None, uses Kt = sqrt(Kp*Ki))
            tau_d: Derivative filter time constant [s]
            Kff: Feedforward gain
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.u_min = u_min
        self.u_max = u_max
        self.tau_d = tau_d
        self.Kff = Kff
        
        # Anti-windup gain (back-calculation method)
        # Rule of thumb: Kt = sqrt(Kp * Ki) if not specified
        if Kt is None:
            self.Kt = np.sqrt(Kp * Ki) if Ki > 0 else 1.0
        else:
            self.Kt = Kt
        
        # Controller state variables
        self.integral = 0.0  # Integral term
        self.error_prev = 0.0  # Previous error for derivative
        self.derivative_filtered = 0.0  # Filtered derivative term
        self.u_unsat = 0.0  # Unsaturated control output
        
        # History for analysis
        self.error_history = []
        self.output_history = []
        self.integral_history = []
        
        # Setpoint rate limiter for bumpless transfer
        self.setpoint_prev = setpoint
        self.setpoint_rate_limit = 50.0  # [bar/s] - maximum setpoint change rate
        
        logger.info(f"PID Controller initialized: Kp={Kp}, Ki={Ki}, Kd={Kd}, "
                   f"Kt={self.Kt:.3f}, tau_d={tau_d}s, Kff={Kff}, "
                   f"output_limits=[{u_min}V, {u_max}V]")
    
    def compute_control_signal(self, current_pressure, dt, feedforward_input=None):
        """
        Compute PID control signal with anti-windup and derivative filtering.
        
        Args:
            current_pressure: Current measured pressure [bar]
            dt: Time step [s]
            feedforward_input: Optional feedforward signal (e.g., setpoint)
            
        Returns:
            u: Control signal [V] (saturated)
        """
        # Apply setpoint rate limiting for bumpless transfer
        setpoint_limited = self._apply_setpoint_rate_limit(dt)
        
        # Compute error
        error = setpoint_limited - current_pressure
        
        # --- Proportional Term ---
        P_term = self.Kp * error
        
        # --- Integral Term with Anti-Windup ---
        # Only integrate if not saturated OR if integration helps
        # (back-calculation anti-windup)
        I_term = self.Ki * self.integral
        
        # --- Derivative Term with Filtering ---
        # Compute raw derivative
        if dt > 0:
            derivative_raw = (error - self.error_prev) / dt
        else:
            derivative_raw = 0.0
        
        # Apply first-order low-pass filter to derivative
        # Filtered derivative: tau_d * d(D_filt)/dt + D_filt = D_raw
        # Discrete: D_filt(k) = alpha * D_raw + (1-alpha) * D_filt(k-1)
        # where alpha = dt / (tau_d + dt)
        if self.tau_d > 0:
            alpha = dt / (self.tau_d + dt)
            self.derivative_filtered = (alpha * derivative_raw + 
                                       (1 - alpha) * self.derivative_filtered)
        else:
            self.derivative_filtered = derivative_raw
        
        D_term = self.Kd * self.derivative_filtered
        
        # --- Feedforward Term ---
        if feedforward_input is not None and self.Kff > 0:
            FF_term = self.Kff * feedforward_input
        else:
            FF_term = 0.0
        
        # --- Compute Unsaturated Output ---
        self.u_unsat = P_term + I_term + D_term + FF_term
        
        # --- Apply Output Saturation ---
        u_sat = np.clip(self.u_unsat, self.u_min, self.u_max)
        
        # --- Anti-Windup: Back-Calculation Method ---
        # If output is saturated, reduce integral term
        # Integral correction: dI/dt = Ki*e - Kt*(u_sat - u_unsat)
        saturation_error = u_sat - self.u_unsat
        
        if dt > 0:
            # Update integral with anti-windup
            integral_increment = error * dt - (self.Kt / self.Ki) * saturation_error * dt
            self.integral += integral_increment
        
        # Update previous error for next derivative calculation
        self.error_prev = error
        
        # Store history
        self.error_history.append(error)
        self.output_history.append(u_sat)
        self.integral_history.append(self.integral)
        
        # Limit history length to prevent memory issues
        max_history = 10000
        if len(self.error_history) > max_history:
            self.error_history = self.error_history[-max_history:]
            self.output_history = self.output_history[-max_history:]
            self.integral_history = self.integral_history[-max_history:]
        
        # Log saturation events
        if abs(saturation_error) > 0.01:
            logger.warning(f"Control output saturated: u_unsat={self.u_unsat:.2f}V, "
                          f"u_sat={u_sat:.2f}V, saturation_error={saturation_error:.2f}V")
        
        return u_sat
    
    def _apply_setpoint_rate_limit(self, dt):
        """
        Apply rate limiting to setpoint changes for bumpless transfer.
        
        Args:
            dt: Time step [s]
            
        Returns:
            setpoint_limited: Rate-limited setpoint [bar]
        """
        if dt <= 0:
            return self.setpoint
        
        # Maximum allowed change in this time step
        max_change = self.setpoint_rate_limit * dt
        
        # Compute actual change
        setpoint_change = self.setpoint - self.setpoint_prev
        
        # Limit the change
        if abs(setpoint_change) > max_change:
            setpoint_limited = self.setpoint_prev + np.sign(setpoint_change) * max_change
        else:
            setpoint_limited = self.setpoint
        
        # Update previous setpoint
        self.setpoint_prev = setpoint_limited
        
        return setpoint_limited
    
    def compute_feedforward(self, setpoint, pressure_model_params):
        """
        Compute feedforward compensation based on pressure model.
        
        For a first-order pressure model: P_ss = K * theta
        Inverse: theta_ff = P_setpoint / K
        
        Args:
            setpoint: Desired pressure [bar]
            pressure_model_params: Dict with 'K' (gain) and other parameters
            
        Returns:
            u_ff: Feedforward control signal [V]
        """
        # Extract model parameters
        K = pressure_model_params.get('K', 3.89)  # [bar/deg]
        max_angle_deg = pressure_model_params.get('max_angle_deg', 180.0)
        
        # Compute required valve angle for desired pressure (steady-state)
        theta_ff_deg = setpoint / K
        
        # Clamp to valve limits
        theta_ff_deg = np.clip(theta_ff_deg, 0.0, max_angle_deg)
        
        # Convert to feedforward signal (this would be valve position command)
        # For now, return normalized value
        u_ff = theta_ff_deg / max_angle_deg
        
        return u_ff
    
    def set_setpoint(self, setpoint):
        """
        Update the pressure setpoint.
        
        Args:
            setpoint: New desired pressure [bar]
        """
        old_setpoint = self.setpoint
        self.setpoint = setpoint
        logger.info(f"Setpoint changed: {old_setpoint:.1f} bar → {setpoint:.1f} bar")
    
    def reset(self):
        """
        Reset controller state (integral, derivative, history).
        
        Use when starting a new control session or after emergency stop.
        """
        self.integral = 0.0
        self.error_prev = 0.0
        self.derivative_filtered = 0.0
        self.u_unsat = 0.0
        self.setpoint_prev = self.setpoint
        self.error_history = []
        self.output_history = []
        self.integral_history = []
        logger.info("PID controller reset")
    
    def get_error_history(self):
        """
        Get the error history for analysis.
        
        Returns:
            errors: List of error values [bar]
        """
        return self.error_history.copy()
    
    def get_state_info(self):
        """
        Get comprehensive controller state information.
        
        Returns:
            dict: Controller state information
        """
        return {
            'setpoint': self.setpoint,
            'setpoint_limited': self.setpoint_prev,
            'error': self.error_prev,
            'integral': self.integral,
            'derivative_filtered': self.derivative_filtered,
            'P_term': self.Kp * self.error_prev,
            'I_term': self.Ki * self.integral,
            'D_term': self.Kd * self.derivative_filtered,
            'u_unsat': self.u_unsat,
            'is_saturated': abs(self.u_unsat) > self.u_max or self.u_unsat < self.u_min
        }
    
    def tune_gains(self, Kp=None, Ki=None, Kd=None):
        """
        Update PID gains (for online tuning).
        
        Args:
            Kp: New proportional gain (None to keep current)
            Ki: New integral gain (None to keep current)
            Kd: New derivative gain (None to keep current)
        """
        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
            # Update anti-windup gain
            self.Kt = np.sqrt(self.Kp * self.Ki) if self.Ki > 0 else 1.0
        if Kd is not None:
            self.Kd = Kd
        
        logger.info(f"PID gains updated: Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}, Kt={self.Kt:.3f}")
