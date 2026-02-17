# Industrial Rotary Valve Pressure Control System

## Overview

This project implements a complete closed-loop pressure control system for an industrial pressurized tube using a rotary valve driven by a DC motor through a gearbox.

## System Description

The system consists of:
- **DC Motor**: 36V motor with gearbox (40:1 ratio, 85% efficiency)
- **Rotary Valve**: 100 kg solid disk valve with 180° angular travel
- **Pressure Tube**: Industrial tube with first-order pressure dynamics
- **PID Controller**: Closed-loop feedback control for pressure regulation

## System Architecture

### Industrial System Block Diagram

```mermaid
graph LR
    A[Power Supply<br/>36V DC] --> B[DC Motor<br/>Kt=0.8 Nm/A<br/>Ke=0.8 V/rad/s<br/>R=1.2Ω]
    B --> C[Gearbox<br/>40:1 Ratio<br/>85% Efficiency]
    C --> D[Rotary Valve<br/>100 kg Disk<br/>r=0.35m]
    D --> E[Pressure Tube<br/>τ=0.8s<br/>Max: 700 bar]
    E --> F[Pressure Sensor]
    F --> G[PID Controller]
    G --> A
    
    style A fill:#e1f5ff,stroke:#333,stroke-width:2px,color:#000
    style B fill:#fff4e1,stroke:#333,stroke-width:2px,color:#000
    style C fill:#ffe1f5,stroke:#333,stroke-width:2px,color:#000
    style D fill:#e1ffe1,stroke:#333,stroke-width:2px,color:#000
    style E fill:#f5e1ff,stroke:#333,stroke-width:2px,color:#000
    style F fill:#ffe1e1,stroke:#333,stroke-width:2px,color:#000
    style G fill:#e1e1ff,stroke:#333,stroke-width:2px,color:#000
```

### Closed-Loop Control System

```mermaid
graph TB
    SP[Setpoint<br/>P_desired] --> SUM[Σ]
    SUM --> PID[PID Controller<br/>Kp, Ki, Kd]
    PID --> MOT[DC Motor<br/>V = RI + Ke·ω]
    MOT --> GEAR[Gearbox<br/>N = 40:1]
    GEAR --> VALVE[Rotary Valve<br/>J·dω/dt = Kt·I - T_friction]
    VALVE --> PRES[Pressure Dynamics<br/>dP/dt = K·θ - P / τ]
    PRES --> SENS[Pressure Sensor]
    SENS --> SUM
    
    style SP fill:#90EE90,stroke:#333,stroke-width:2px,color:#000
    style SUM fill:#FFB6C1,stroke:#333,stroke-width:2px,color:#000
    style PID fill:#87CEEB,stroke:#333,stroke-width:2px,color:#000
    style MOT fill:#FFD700,stroke:#333,stroke-width:2px,color:#000
    style GEAR fill:#DDA0DD,stroke:#333,stroke-width:2px,color:#000
    style VALVE fill:#F0E68C,stroke:#333,stroke-width:2px,color:#000
    style PRES fill:#98FB98,stroke:#333,stroke-width:2px,color:#000
    style SENS fill:#FFA07A,stroke:#333,stroke-width:2px,color:#000
```

### 4th-Order State-Space Model

```mermaid
graph TD
    subgraph "State Variables"
        X1[x₁: Motor Current I]
        X2[x₂: Motor Speed ω]
        X3[x₃: Valve Angle θ]
        X4[x₄: Pressure P]
    end
    
    subgraph "Differential Equations"
        EQ1[dx₁/dt = V - R·I - Ke·ω / L]
        EQ2[dx₂/dt = Kt·I - T_load / J_eq]
        EQ3[dx₃/dt = ω / N]
        EQ4[dx₄/dt = K·θ - P / τ]
    end
    
    X1 --> EQ1
    X2 --> EQ2
    X3 --> EQ3
    X4 --> EQ4
    
    EQ1 --> X1
    EQ2 --> X2
    EQ3 --> X3
    EQ4 --> X4
    
    style X1 fill:#FFE4B5,stroke:#333,stroke-width:2px,color:#000
    style X2 fill:#E0BBE4,stroke:#333,stroke-width:2px,color:#000
    style X3 fill:#FFDAB9,stroke:#333,stroke-width:2px,color:#000
    style X4 fill:#B0E0E6,stroke:#333,stroke-width:2px,color:#000
```

### Physical System Components

```mermaid
graph TB
    subgraph "Motor Parameters"
        M1[Voltage: 36V DC]
        M2[Torque Constant: 0.8 Nm/A]
        M3[Back-EMF: 0.8 V/rad/s]
        M4[Resistance: 1.2Ω]
        M5[Inductance: 0.05 H]
    end
    
    subgraph "Valve Parameters"
        V1[Mass: 100 kg]
        V2[Radius: 0.35 m]
        V3[Inertia: 6.125 kg·m²]
        V4[Friction: 120 Nm]
        V5[Travel: 0° to 180°]
    end
    
    subgraph "Pressure Model"
        P1[Time Constant: 0.8 s]
        P2[Max Pressure: 700 bar]
        P3[Gain: 3.89 bar/deg]
    end
    
    subgraph "Control Requirements"
        C1[Settling Time: < 3s]
        C2[Overshoot: < 10%]
        C3[SS Error: < 2%]
    end
    
    style M1 fill:#FFE4E1,stroke:#333,stroke-width:2px,color:#000
    style V1 fill:#E0FFE0,stroke:#333,stroke-width:2px,color:#000
    style P1 fill:#E0E0FF,stroke:#333,stroke-width:2px,color:#000
    style C1 fill:#FFFFE0,stroke:#333,stroke-width:2px,color:#000
```


## Project Structure

```
pressure_regulation/
├── models/              # Physical system models
│   ├── motor.py        # DC motor electrical/mechanical dynamics
│   ├── valve.py        # Rotary valve mechanics
│   └── pressure.py     # Tube pressure dynamics
├── control/            # Control algorithms
│   └── pid.py          # PID controller implementation
├── simulation/         # Simulation engine
│   └── simulator.py    # 4th-order system simulator
├── gui/                # Graphical user interface
│   └── dashboard.py    # Tkinter-based dashboard
├── config/             # Configuration files
│   └── parameters.json # System parameters
├── utils/              # Utility modules
│   └── performance_metrics.py  # Performance analysis
├── main.py             # Main entry point
├── requirements.txt    # Python dependencies
└── README.md          # This file
```

### Software Architecture Diagram

```mermaid
graph TB
    subgraph "User Interface Layer"
        GUI[GUI Dashboard<br/>Tkinter]
    end
    
    subgraph "Control Layer"
        PID[PID Controller<br/>pid.py]
    end
    
    subgraph "Simulation Layer"
        SIM[Simulator<br/>simulator.py]
    end
    
    subgraph "Model Layer"
        MOTOR[Motor Model<br/>motor.py]
        VALVE[Valve Model<br/>valve.py]
        PRESSURE[Pressure Model<br/>pressure.py]
    end
    
    subgraph "Configuration Layer"
        CONFIG[Parameters<br/>parameters.json]
        UTILS[Performance Metrics<br/>performance_metrics.py]
    end
    
    GUI --> SIM
    GUI --> CONFIG
    SIM --> PID
    SIM --> MOTOR
    SIM --> VALVE
    SIM --> PRESSURE
    PID --> CONFIG
    MOTOR --> CONFIG
    VALVE --> CONFIG
    PRESSURE --> CONFIG
    SIM --> UTILS
    
    style GUI fill:#87CEEB,stroke:#333,stroke-width:2px,color:#000
    style PID fill:#98FB98,stroke:#333,stroke-width:2px,color:#000
    style SIM fill:#FFD700,stroke:#333,stroke-width:2px,color:#000
    style MOTOR fill:#FFB6C1,stroke:#333,stroke-width:2px,color:#000
    style VALVE fill:#DDA0DD,stroke:#333,stroke-width:2px,color:#000
    style PRESSURE fill:#F0E68C,stroke:#333,stroke-width:2px,color:#000
    style CONFIG fill:#FFA07A,stroke:#333,stroke-width:2px,color:#000
    style UTILS fill:#E0BBE4,stroke:#333,stroke-width:2px,color:#000
```

### Data Flow Architecture

The system follows this execution flow:

1. **User Input**: User sets pressure setpoint via GUI
2. **Initialization**: GUI initializes simulator with system parameters
3. **Simulation Loop**: 
   - Simulator requests state derivatives from physical models
   - Models return dx/dt based on current states
   - PID controller computes control voltage based on pressure error
   - Simulator integrates using `scipy.integrate.solve_ivp`
   - GUI updates real-time displays
4. **Performance Analysis**: Metrics module computes performance statistics
5. **Results Display**: GUI shows final performance report to user


## Installation

1. **Clone or download this repository**

2. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Verify installation:**
   ```bash
   python -c "import numpy, scipy, matplotlib; print('Dependencies OK')"
   ```

## Usage

Run the application:
```bash
python main.py
```

The GUI dashboard will open, allowing you to:
- Set desired pressure setpoint
- Start/stop simulation
- Monitor real-time pressure, valve angle, and motor current
- View live pressure graph

## Control Requirements

The PID controller is designed to meet:
- **Settling time**: < 3 seconds
- **Overshoot**: < 10%
- **Steady-state error**: < 2%
- **Disturbance rejection**: ±50 bar step disturbance

## System Parameters

All parameters are defined in `config/parameters.json`:
- Motor: 36V, Kt=0.8 Nm/A, Ke=0.8 V/(rad/s), R=1.2Ω
- Valve: 100 kg, radius=0.35m, friction=120 Nm
- Pressure: τ=0.8s, max=700 bar
- Gear: 40:1 ratio, 85% efficiency

## Mathematical Model

The system is a **4th-order nonlinear dynamic system**:

**State variables:**
- x₁: Motor current (I)
- x₂: Motor speed (ω)
- x₃: Valve angle (θ)
- x₄: Pressure (P)

**Governing equations:**
1. Electrical: `V = R·I + Ke·ω`
2. Mechanical: `J_eq·(dω/dt) = Kt·I - T_load`
3. Kinematics: `dθ/dt = ω / gear_ratio`
4. Pressure: `dP/dt = (K·θ - P) / τ`

## Performance Validation

The system automatically computes:
- Settling time
- Percent overshoot
- Steady-state error
- Stability confirmation
- Disturbance recovery time

## Safety Architecture

This system implements industry-standard safety monitoring suitable for aerospace/avionics applications.

```mermaid
graph TB
    subgraph "Safety Monitoring Layer"
        MOTOR_SAFE[Motor Safety<br/>Overcurrent, Stall, Thermal]
        PRESS_SAFE[Pressure Safety<br/>Overpressure, Rapid Change]
        VALVE_SAFE[Valve Safety<br/>Position Limits, Jam Detection]
    end
    
    subgraph "Validation Layer"
        SENSOR_VAL[Sensor Validation<br/>Range Check, Fallback]
    end
    
    subgraph "Control Layer"
        SAFETY_MGR[Safety Manager]
        ESTOP[Emergency Stop Controller]
    end
    
    subgraph "Physical System"
        MOTOR[Motor]
        VALVE[Valve]
        PRESSURE[Pressure System]
    end
    
    MOTOR --> MOTOR_SAFE
    VALVE --> VALVE_SAFE
    PRESSURE --> PRESS_SAFE
    
    MOTOR_SAFE --> SENSOR_VAL
    VALVE_SAFE --> SENSOR_VAL
    PRESS_SAFE --> SENSOR_VAL
    
    SENSOR_VAL --> SAFETY_MGR
    SAFETY_MGR --> ESTOP
    
    ESTOP --> MOTOR
    ESTOP --> VALVE
    
    style MOTOR_SAFE fill:#FFB6C1,stroke:#333,stroke-width:2px,color:#000
    style PRESS_SAFE fill:#98FB98,stroke:#333,stroke-width:2px,color:#000
    style VALVE_SAFE fill:#87CEEB,stroke:#333,stroke-width:2px,color:#000
    style SENSOR_VAL fill:#FFD700,stroke:#333,stroke-width:2px,color:#000
    style SAFETY_MGR fill:#DDA0DD,stroke:#333,stroke-width:2px,color:#000
    style ESTOP fill:#FF6B6B,stroke:#333,stroke-width:2px,color:#000
```

## Safety Considerations

### Safety Monitoring Subsystems

1. **Motor Safety Monitor**
   - **Overcurrent Protection**: Triggers at >30A (instantaneous)
   - **Stall Detection**: Triggers at >25A for >0.5s
   - **Thermal Protection**: I²t monitoring with 60s time constant
   - **Sensor Validation**: Rejects readings outside [-1A, 35A]

2. **Pressure Safety Monitor**
   - **Overpressure Protection**: Critical at >700 bar, warning at >650 bar
   - **Underpressure Protection**: Triggers at <0 bar
   - **Rapid Change Detection**: Triggers at >200 bar/s (potential rupture/leak)
   - **Sensor Validation**: Rejects readings outside [-10 bar, 750 bar]

3. **Valve Safety Monitor**
   - **Position Limits**: Enforces 0° to 180° range
   - **Velocity Limits**: Triggers at >90°/s
   - **Jam Detection**: Detects commanded motion with no actual movement
   - **Sensor Validation**: Rejects readings outside [-10°, 190°]

4. **Emergency Stop Controller**
   - **Immediate Shutdown**: Motor voltage → 0V in <0.5s
   - **Valve Hold**: Maintains current position during stop
   - **Manual Reset Required**: Operator intervention needed after safety fault
   - **State Preservation**: Logs fault reason and system state

### Sensor Validation

All sensors implement validation with fallback behavior:

| Sensor | Valid Range | Fallback Value | Failure Threshold |
|--------|-------------|----------------|-------------------|
| Pressure | -10 to 750 bar | 100 bar (safe mid-range) | 3 consecutive out-of-range |
| Motor Current | -1 to 35 A | 0 A (motor off) | 3 consecutive out-of-range |
| Valve Position | -10° to 190° | 90° (mid-position) | 3 consecutive out-of-range |

**Fallback Strategy:**
1. First out-of-range reading → Use last valid reading, log warning
2. Second consecutive → Continue using last valid, increase warning level
3. Third consecutive → Sensor declared failed, use configured fallback value

## Fault Cases & Mitigation Strategies

| Fault Type | Detection Method | Mitigation Strategy | Recovery |
|------------|------------------|---------------------|----------|
| **Motor Overcurrent** | Current >30A | Emergency stop, cut motor power | Manual reset after fault cleared |
| **Motor Stall** | Current >25A for >0.5s | Emergency stop, prevent thermal damage | Manual reset, check mechanical jam |
| **Motor Thermal Limit** | I²t accumulator >1.2× rated | Emergency stop, cooldown required | Automatic after thermal decay |
| **Overpressure** | Pressure >700 bar | Emergency stop, valve to safe position | Manual reset, check pressure relief |
| **Rapid Pressure Change** | dP/dt >200 bar/s | Emergency stop, potential rupture | Manual inspection required |
| **Valve Position Limit** | Angle <0° or >180° | Emergency stop, mechanical stop engaged | Manual reset, check position sensor |
| **Valve Jam** | Commanded motion but no movement | Emergency stop, mechanical fault | Manual inspection, clear obstruction |
| **Sensor Failure** | 3 consecutive out-of-range | Use fallback value, log error | Sensor replacement, manual reset |

## Logging

The system implements structured logging with severity levels:

- **DEBUG**: Detailed diagnostic information
- **INFO**: Normal operation events (setpoint changes, mode transitions)
- **WARN**: Abnormal conditions that don't require shutdown (approaching limits)
- **ERROR**: Fault conditions requiring emergency stop

**Log Files:**
- `logs/pressure_control.log`: All events (rotating, 10MB max, 5 backups)
- `logs/errors.log`: ERROR level and above only

**Logged Events:**
- Control saturation (PID output at limits)
- Motor stall warnings
- Sensor faults and fallback activation
- Safety violations and emergency stops
- System state transitions

## Testing

### Running Tests

**Install test dependencies:**
```bash
pip install pytest pytest-cov pytest-mock
```

**Run all tests with coverage:**
```bash
pytest
```

**Run specific test categories:**
```bash
# Safety-critical tests only
pytest -m safety

# Unit tests only
pytest -m unit

# Integration tests only
pytest -m integration
```

**Run specific test files:**
```bash
pytest tests/test_motor_safety.py -v
pytest tests/test_sensor_validation.py -v
pytest tests/test_safety_integration.py -v
```

**Generate coverage report:**
```bash
pytest --cov=. --cov-report=html
# Open htmlcov/index.html in browser
```

### Test Structure

```
tests/
├── conftest.py              # Test fixtures and configuration
├── test_motor_safety.py     # Motor safety monitor tests
├── test_sensor_validation.py # Sensor validation tests
└── test_safety_integration.py # Safety manager integration tests
```

**Coverage Goals:**
- Safety modules: >90% coverage
- Validation modules: >85% coverage
- Control modules: >80% coverage

## Technology Stack

- **Language**: Python 3.8+
- **Numerical**: NumPy, SciPy
- **Visualization**: Matplotlib
- **GUI**: Tkinter
- **Integration**: scipy.integrate.solve_ivp

## Author

Industrial Pressure Control System  
Date: 2026-02-18

## License

Educational/Research Use
