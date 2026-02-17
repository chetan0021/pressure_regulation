# Industrial Rotary Valve Pressure Control System

## Knowledge Base & Engineering Specification

*(Python Implementation Only)*

------------------------------------------------------------------------

# 1. Problem Overview

An industrial pressurized tube requires accurate pressure regulation
using a rotary valve driven by a DC motor through a gearbox.

The system operates in closed-loop using pressure feedback.

The solution must include:

-   Numerical actuator design
-   Dynamic system modeling
-   Closed-loop PID control
-   Simulation
-   GUI communication
-   Performance verification

MATLAB is NOT allowed.\
Python must be used.

------------------------------------------------------------------------

# 2. Given System Parameters

## Valve

-   Mass = 100 kg\
-   Radius (shaft to center of mass) = 0.35 m\
-   Static friction torque = 120 Nm\
-   Angular travel = 0° to 180°\
-   Assume solid disk

### Moment of Inertia

J = 1/2 m r\^2\
J = 6.125 kg·m\^2

------------------------------------------------------------------------

## Gravity Torque

T_g = m g r\
T_g = 343.35 Nm

------------------------------------------------------------------------

## Total Load Torque at Shaft

T_total = T_g + T_f\
T_total = 463.35 Nm

------------------------------------------------------------------------

# 3. Motor Parameters

-   Supply voltage = 36 V\
-   Torque constant Kt = 0.8 Nm/A\
-   Back EMF constant Ke = 0.8 V/(rad/s)\
-   Armature resistance R = 1.2 Ohm\
-   Gear ratio = 40:1\
-   Gear efficiency = 85%

------------------------------------------------------------------------

## Required Motor Torque

T_motor = T_load / (gear_ratio × efficiency)\
T_motor = 13.63 Nm

------------------------------------------------------------------------

## Required Motor Current

I = T / Kt\
I = 17.04 A

------------------------------------------------------------------------

# 4. Dynamic System Modeling

## 4.1 Electrical Model (DC Motor)

Assuming negligible inductance:

V = R I + Ke ω

Where: - V = input voltage - I = motor current - ω = motor angular speed

------------------------------------------------------------------------

## 4.2 Mechanical Model

J_eq dω/dt = Kt I - T_load

Reflected inertia:

J_eq = J_valve / (gear_ratio)\^2

------------------------------------------------------------------------

## 4.3 Valve Angle

dθ/dt = ω / gear_ratio

------------------------------------------------------------------------

## 4.4 Pressure Model

Assume first-order tube pressure dynamics:

dP/dt = (Kθ - P) / τ

Where:

-   P = tube pressure (bar)
-   θ = valve angle
-   τ = time constant (assume 0.8 s)
-   K = pressure gain

Assume: - 180° → 700 bar

------------------------------------------------------------------------

# 5. State Variables

Define system states:

-   x1 = Motor current (I)
-   x2 = Motor speed (ω)
-   x3 = Valve angle (θ)
-   x4 = Pressure (P)

This creates a 4th-order nonlinear system.

------------------------------------------------------------------------

# 6. Control System Requirements

Design PID controller such that:

-   Settling time \< 3 s
-   Overshoot \< 10%
-   Steady-state error \< 2%
-   Disturbance rejection for ±50 bar step disturbance

------------------------------------------------------------------------

# 7. Simulation Requirements

Use Python libraries:

-   numpy
-   scipy
-   matplotlib
-   control (optional)

Simulation must:

-   Run for 10 seconds
-   Use scipy.integrate.solve_ivp
-   Inject disturbance at t = 4 s
-   Log:
    -   Pressure vs time
    -   Valve angle vs time
    -   Motor current vs time

------------------------------------------------------------------------

# 8. GUI Requirements

GUI must:

-   Display real-time pressure
-   Display valve angle
-   Display motor current
-   Allow user to enter pressure setpoint
-   Start/Stop simulation
-   Plot pressure graph live

Preferred: Tkinter

------------------------------------------------------------------------

# 9. Required Python Project Structure

pressure_control_system/

│ ├── models/ │ ├── motor.py │ ├── valve.py │ ├── pressure.py │ ├──
control/ │ ├── pid.py │ ├── simulation/ │ ├── simulator.py │ ├── gui/ │
├── dashboard.py │ ├── config/ │ ├── parameters.json │ ├── utils/ │ ├──
performance_metrics.py │ └── main.py

------------------------------------------------------------------------

# 10. Performance Evaluation Metrics

The system must automatically compute:

-   Settling time
-   Percent overshoot
-   Steady-state error
-   Stability confirmation
-   Disturbance recovery time

------------------------------------------------------------------------

# 11. Engineering Validation Goals

The final implementation must demonstrate:

-   Correct actuator torque sizing
-   Stable closed-loop behavior
-   Compliance with time-domain specifications
-   Realistic physical modeling
-   Clean modular architecture
-   Separation of models, control, simulation, and GUI

------------------------------------------------------------------------

# 12. Engineering Assumptions

The following assumptions are acceptable:

-   Neglect motor inductance
-   Constant gear efficiency
-   First-order pressure dynamics
-   Linear valve-pressure relationship
-   No nonlinear saturation (unless added later)

------------------------------------------------------------------------

# 13. Objective of This Document

This document provides:

-   Required theoretical knowledge
-   Numerical foundation
-   Mathematical modeling
-   Control requirements
-   Software architecture expectations

Any AI agent using this document must:

1.  Learn the system physics
2.  Follow provided equations
3.  Maintain modular code architecture
4.  Implement simulation in Python only
5.  Generate GUI interface
6.  Validate control performance

------------------------------------------------------------------------

# End of Knowledge Base
