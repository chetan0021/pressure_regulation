"""
Generate Architecture Diagrams for README

This script creates visual diagrams for the pressure control system documentation.
Run this script to generate diagram images that will be embedded in README.md

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import os


def create_industrial_system_diagram():
    """Create the industrial system block diagram."""
    fig, ax = plt.subplots(figsize=(14, 8))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8)
    ax.axis('off')
    
    # Define components
    components = [
        {'name': 'Power Supply\n36V DC', 'pos': (1, 4), 'color': '#ff9999'},
        {'name': 'DC Motor\nKt=0.8 Nm/A\nR=1.2Ω', 'pos': (3.5, 4), 'color': '#99ccff'},
        {'name': 'Gearbox\n40:1 Ratio\n85% Efficiency', 'pos': (6, 4), 'color': '#99ff99'},
        {'name': 'Rotary Valve\n100 kg\n0°-180°', 'pos': (8.5, 4), 'color': '#ffcc99'},
        {'name': 'Pressure Tube\n0-700 bar', 'pos': (11, 4), 'color': '#cc99ff'},
    ]
    
    feedback = [
        {'name': 'Pressure\nSensor', 'pos': (11, 1.5), 'color': '#ffff99'},
        {'name': 'PID\nController', 'pos': (1, 1.5), 'color': '#ff99cc'},
    ]
    
    # Draw forward path components
    for comp in components:
        box = FancyBboxPatch((comp['pos'][0]-0.6, comp['pos'][1]-0.5), 1.2, 1,
                            boxstyle="round,pad=0.1", 
                            edgecolor='black', facecolor=comp['color'], linewidth=2)
        ax.add_patch(box)
        ax.text(comp['pos'][0], comp['pos'][1], comp['name'], 
               ha='center', va='center', fontsize=9, weight='bold')
    
    # Draw feedback path components
    for comp in feedback:
        box = FancyBboxPatch((comp['pos'][0]-0.6, comp['pos'][1]-0.5), 1.2, 1,
                            boxstyle="round,pad=0.1", 
                            edgecolor='black', facecolor=comp['color'], linewidth=2)
        ax.add_patch(box)
        ax.text(comp['pos'][0], comp['pos'][1], comp['name'], 
               ha='center', va='center', fontsize=9, weight='bold')
    
    # Draw arrows (forward path)
    arrow_props = dict(arrowstyle='->', lw=2.5, color='black')
    for i in range(len(components)-1):
        ax.annotate('', xy=(components[i+1]['pos'][0]-0.7, components[i+1]['pos'][1]),
                   xytext=(components[i]['pos'][0]+0.7, components[i]['pos'][1]),
                   arrowprops=arrow_props)
    
    # Feedback arrows
    ax.annotate('', xy=(feedback[1]['pos'][0], feedback[1]['pos'][1]+0.5),
               xytext=(feedback[1]['pos'][0], components[0]['pos'][1]-0.5),
               arrowprops=arrow_props)
    
    ax.annotate('', xy=(feedback[1]['pos'][0]-0.6, feedback[1]['pos'][1]),
               xytext=(feedback[0]['pos'][0]+0.6, feedback[0]['pos'][1]),
               arrowprops=dict(arrowstyle='->', lw=2.5, color='black',
                             connectionstyle="arc3,rad=-.3"))
    
    ax.annotate('', xy=(feedback[0]['pos'][0], components[-1]['pos'][1]-0.5),
               xytext=(feedback[0]['pos'][0], feedback[0]['pos'][1]+0.5),
               arrowprops=arrow_props)
    
    plt.title('Industrial Pressure Control System - Block Diagram', 
             fontsize=16, weight='bold', pad=20)
    
    plt.tight_layout()
    plt.savefig('docs/industrial_system_diagram.png', dpi=300, bbox_inches='tight')
    print("[OK] Created: industrial_system_diagram.png")
    plt.close()


def create_control_loop_diagram():
    """Create the closed-loop control system diagram."""
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 10)
    ax.axis('off')
    
    # Components with positions
    components = {
        'setpoint': {'pos': (1, 8), 'text': 'Setpoint\nDesired\nPressure', 'color': '#90EE90'},
        'sum': {'pos': (3, 8), 'text': '±', 'color': '#FFD700'},
        'pid': {'pos': (5, 8), 'text': 'PID Controller\nKp, Ki, Kd', 'color': '#FF6B6B'},
        'motor': {'pos': (8, 8), 'text': 'DC Motor\nV = RI + Keω', 'color': '#4ECDC4'},
        'gearbox': {'pos': (11, 8), 'text': 'Gearbox\n40:1', 'color': '#95E1D3'},
        'valve': {'pos': (11, 5), 'text': 'Rotary Valve\nJ·dω/dt=KtI-Tload', 'color': '#F38181'},
        'pressure': {'pos': (8, 2), 'text': 'Pressure\nDynamics\ndP/dt=(Kθ-P)/τ', 'color': '#AA96DA'},
        'sensor': {'pos': (3, 2), 'text': 'Pressure\nSensor', 'color': '#FCBAD3'},
        'disturbance': {'pos': (11, 2), 'text': 'Disturbance\n±50 bar', 'color': '#FFA07A'},
    }
    
    # Draw components
    for key, comp in components.items():
        if key == 'sum':
            circle = plt.Circle(comp['pos'], 0.4, edgecolor='black', 
                              facecolor=comp['color'], linewidth=2)
            ax.add_patch(circle)
            ax.text(comp['pos'][0], comp['pos'][1], comp['text'], 
                   ha='center', va='center', fontsize=14, weight='bold')
        else:
            box = FancyBboxPatch((comp['pos'][0]-0.7, comp['pos'][1]-0.5), 1.4, 1,
                                boxstyle="round,pad=0.1", 
                                edgecolor='black', facecolor=comp['color'], linewidth=2)
            ax.add_patch(box)
            ax.text(comp['pos'][0], comp['pos'][1], comp['text'], 
                   ha='center', va='center', fontsize=8, weight='bold')
    
    # Draw arrows
    arrow_props = dict(arrowstyle='->', lw=2, color='black')
    
    # Forward path
    ax.annotate('', xy=(2.6, 8), xytext=(1.7, 8), arrowprops=arrow_props)
    ax.annotate('', xy=(4.3, 8), xytext=(3.4, 8), arrowprops=arrow_props)
    ax.annotate('', xy=(7.3, 8), xytext=(5.7, 8), arrowprops=arrow_props)
    ax.annotate('', xy=(10.3, 8), xytext=(8.7, 8), arrowprops=arrow_props)
    ax.annotate('', xy=(11, 7.5), xytext=(11, 8.5), arrowprops=arrow_props)
    ax.annotate('', xy=(11, 5.5), xytext=(11, 6.5), arrowprops=arrow_props)
    ax.annotate('', xy=(8.7, 2), xytext=(10.3, 2), arrowprops=arrow_props)
    ax.annotate('', xy=(3.7, 2), xytext=(7.3, 2), arrowprops=arrow_props)
    
    # Feedback path
    ax.annotate('', xy=(3, 7.6), xytext=(3, 2.5), arrowprops=arrow_props)
    
    # Disturbance (dashed)
    ax.annotate('', xy=(8.7, 2.3), xytext=(10.3, 2.3), 
               arrowprops=dict(arrowstyle='->', lw=2, color='red', linestyle='dashed'))
    
    plt.title('Closed-Loop Pressure Control System', 
             fontsize=16, weight='bold', pad=20)
    
    plt.tight_layout()
    plt.savefig('docs/control_loop_diagram.png', dpi=300, bbox_inches='tight')
    print("[OK] Created: control_loop_diagram.png")
    plt.close()


def create_software_architecture():
    """Create the software architecture diagram."""
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 10)
    ax.axis('off')
    
    layers = [
        {'name': 'User Interface Layer', 'components': ['GUI Dashboard\n(Tkinter)'], 
         'y': 8.5, 'color': '#FF6B6B'},
        {'name': 'Control Layer', 'components': ['PID Controller\n(pid.py)'], 
         'y': 7, 'color': '#4ECDC4'},
        {'name': 'Simulation Layer', 'components': ['System Simulator\n(scipy.integrate.solve_ivp)'], 
         'y': 5.5, 'color': '#45B7D1'},
        {'name': 'Model Layer', 'components': ['DC Motor\n(motor.py)', 'Rotary Valve\n(valve.py)', 'Pressure\n(pressure.py)'], 
         'y': 4, 'color': ['#96CEB4', '#FFEAA7', '#DFE6E9']},
        {'name': 'Configuration Layer', 'components': ['Parameters\n(parameters.json)'], 
         'y': 2.5, 'color': '#A29BFE'},
        {'name': 'Analysis Layer', 'components': ['Performance Metrics\n(performance_metrics.py)'], 
         'y': 1, 'color': '#FD79A8'},
    ]
    
    for layer in layers:
        # Draw layer label
        ax.text(0.5, layer['y'], layer['name'], fontsize=10, weight='bold', 
               rotation=90, va='center', ha='center')
        
        # Draw components
        n_comp = len(layer['components'])
        spacing = 10 / (n_comp + 1)
        
        for i, comp in enumerate(layer['components']):
            x = 2 + spacing * (i + 1)
            color = layer['color'][i] if isinstance(layer['color'], list) else layer['color']
            
            box = FancyBboxPatch((x-0.8, layer['y']-0.35), 1.6, 0.7,
                                boxstyle="round,pad=0.05", 
                                edgecolor='black', facecolor=color, linewidth=2)
            ax.add_patch(box)
            ax.text(x, layer['y'], comp, ha='center', va='center', 
                   fontsize=8, weight='bold')
    
    # Draw arrows between layers
    arrow_props = dict(arrowstyle='->', lw=2, color='#333333')
    for i in range(len(layers)-1):
        ax.annotate('', xy=(6, layers[i+1]['y']+0.35), 
                   xytext=(6, layers[i]['y']-0.35),
                   arrowprops=arrow_props)
    
    plt.title('Software Architecture - Layered Design', 
             fontsize=16, weight='bold', pad=20)
    
    plt.tight_layout()
    plt.savefig('docs/software_architecture.png', dpi=300, bbox_inches='tight')
    print("[OK] Created: software_architecture.png")
    plt.close()


def create_system_specifications():
    """Create system specifications diagram."""
    fig, ax = plt.subplots(figsize=(14, 8))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8)
    ax.axis('off')
    
    # Valve Mechanics
    valve_box = FancyBboxPatch((0.5, 4), 4, 3.5, boxstyle="round,pad=0.1",
                              edgecolor='black', facecolor='#FFE4B5', linewidth=2)
    ax.add_patch(valve_box)
    ax.text(2.5, 7.2, 'Valve Mechanics', ha='center', fontsize=12, weight='bold')
    
    valve_specs = [
        'Mass: 100 kg',
        'Radius: 0.35 m',
        'Inertia J = 6.125 kg·m²',
        'Friction: 120 Nm',
        'Gravity: 343.35 Nm',
        'Total Load: 463.35 Nm'
    ]
    
    for i, spec in enumerate(valve_specs):
        ax.text(2.5, 6.7 - i*0.5, spec, ha='center', fontsize=10)
    
    # Motor Specs
    motor_box = FancyBboxPatch((5, 4), 4, 3.5, boxstyle="round,pad=0.1",
                              edgecolor='black', facecolor='#B0E0E6', linewidth=2)
    ax.add_patch(motor_box)
    ax.text(7, 7.2, 'Motor Specifications', ha='center', fontsize=12, weight='bold')
    
    motor_specs = [
        'Supply: 36V',
        'Kt: 0.8 Nm/A',
        'Ke: 0.8 V/(rad/s)',
        'R: 1.2 Ω',
        'Required I: 17.04 A',
        'Required T: 13.63 Nm'
    ]
    
    for i, spec in enumerate(motor_specs):
        ax.text(7, 6.7 - i*0.5, spec, ha='center', fontsize=10)
    
    # Pressure Model
    pressure_box = FancyBboxPatch((9.5, 4), 4, 3.5, boxstyle="round,pad=0.1",
                                 edgecolor='black', facecolor='#DDA0DD', linewidth=2)
    ax.add_patch(pressure_box)
    ax.text(11.5, 7.2, 'Pressure Model', ha='center', fontsize=12, weight='bold')
    
    pressure_specs = [
        'Time Constant: 0.8 s',
        'Max Pressure: 700 bar',
        'Linear: 180° → 700 bar',
        '',
        'dP/dt = (K·θ - P) / τ',
        ''
    ]
    
    for i, spec in enumerate(pressure_specs):
        ax.text(11.5, 6.7 - i*0.5, spec, ha='center', fontsize=10)
    
    # Control Requirements
    control_box = FancyBboxPatch((0.5, 0.2), 13, 3.3, boxstyle="round,pad=0.1",
                                edgecolor='black', facecolor='#E0FFE0', linewidth=2)
    ax.add_patch(control_box)
    ax.text(7, 3.2, 'Control Performance Requirements', ha='center', fontsize=12, weight='bold')
    
    control_specs = [
        'Settling Time: < 3 seconds  |  Overshoot: < 10%  |  Steady-State Error: < 2%',
        'Disturbance Rejection: ±50 bar step disturbance at t = 4s',
        'Controller Type: PID (Proportional-Integral-Derivative)'
    ]
    
    for i, spec in enumerate(control_specs):
        ax.text(7, 2.5 - i*0.5, spec, ha='center', fontsize=10)
    
    plt.title('System Specifications & Requirements', 
             fontsize=16, weight='bold', pad=20)
    
    plt.tight_layout()
    plt.savefig('docs/system_specifications.png', dpi=300, bbox_inches='tight')
    print("[OK] Created: system_specifications.png")
    plt.close()


def create_state_space_model():
    """Create state-space model visualization."""
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_xlim(0, 12)
    ax.set_ylim(0, 8)
    ax.axis('off')
    
    # State variables
    states = [
        {'name': 'x₁\nMotor Current\n(I)', 'pos': (2, 6), 'color': '#FFB6C1'},
        {'name': 'x₂\nMotor Speed\n(ω)', 'pos': (5, 6), 'color': '#87CEEB'},
        {'name': 'x₃\nValve Angle\n(θ)', 'pos': (8, 6), 'color': '#98FB98'},
        {'name': 'x₄\nPressure\n(P)', 'pos': (11, 6), 'color': '#DDA0DD'},
    ]
    
    # Equations
    equations = [
        {'eq': 'dx₁/dt =\n(V - RI - Keω) / L', 'pos': (2, 3), 'color': '#FFB6C1'},
        {'eq': 'dx₂/dt =\n(KtI - Tload) / Jeq', 'pos': (5, 3), 'color': '#87CEEB'},
        {'eq': 'dx₃/dt =\nω / gear_ratio', 'pos': (8, 3), 'color': '#98FB98'},
        {'eq': 'dx₄/dt =\n(K·θ - P) / τ', 'pos': (11, 3), 'color': '#DDA0DD'},
    ]
    
    # Draw state boxes
    for state in states:
        box = FancyBboxPatch((state['pos'][0]-0.7, state['pos'][1]-0.5), 1.4, 1,
                            boxstyle="round,pad=0.1", 
                            edgecolor='black', facecolor=state['color'], linewidth=2)
        ax.add_patch(box)
        ax.text(state['pos'][0], state['pos'][1], state['name'], 
               ha='center', va='center', fontsize=9, weight='bold')
    
    # Draw equation boxes
    for eq in equations:
        box = FancyBboxPatch((eq['pos'][0]-0.9, eq['pos'][1]-0.5), 1.8, 1,
                            boxstyle="round,pad=0.1", 
                            edgecolor='black', facecolor=eq['color'], linewidth=2, alpha=0.6)
        ax.add_patch(box)
        ax.text(eq['pos'][0], eq['pos'][1], eq['eq'], 
               ha='center', va='center', fontsize=8, weight='bold')
    
    # Draw arrows
    arrow_props = dict(arrowstyle='->', lw=2, color='black')
    for i in range(4):
        # State to equation
        ax.annotate('', xy=(states[i]['pos'][0], equations[i]['pos'][1]+0.5),
                   xytext=(states[i]['pos'][0], states[i]['pos'][1]-0.5),
                   arrowprops=arrow_props)
        
        # Equation back to state (feedback)
        ax.annotate('', xy=(states[i]['pos'][0]+0.5, states[i]['pos'][1]-0.3),
                   xytext=(equations[i]['pos'][0]+0.5, equations[i]['pos'][1]+0.3),
                   arrowprops=dict(arrowstyle='->', lw=1.5, color='red', linestyle='dashed',
                                 connectionstyle="arc3,rad=.5"))
    
    # Title and labels
    ax.text(6, 7.5, '4th-Order State-Space Model', ha='center', 
           fontsize=16, weight='bold')
    ax.text(6, 1.5, 'State Variables', ha='center', fontsize=12, weight='bold')
    ax.text(6, 1, 'Differential Equations (System Dynamics)', ha='center', fontsize=12, weight='bold')
    
    plt.tight_layout()
    plt.savefig('docs/state_space_model.png', dpi=300, bbox_inches='tight')
    print("[OK] Created: state_space_model.png")
    plt.close()


def main():
    """Generate all diagrams."""
    # Create docs directory if it doesn't exist
    os.makedirs('docs', exist_ok=True)
    
    print("\n" + "="*60)
    print("Generating Architecture Diagrams")
    print("="*60 + "\n")
    
    create_industrial_system_diagram()
    create_control_loop_diagram()
    create_software_architecture()
    create_system_specifications()
    create_state_space_model()
    
    print("\n" + "="*60)
    print("[OK] All diagrams generated successfully!")
    print("="*60)
    print("\nDiagrams saved in 'docs/' directory:")
    print("  - industrial_system_diagram.png")
    print("  - control_loop_diagram.png")
    print("  - software_architecture.png")
    print("  - system_specifications.png")
    print("  - state_space_model.png")
    print("\nYou can now view these images in the README.md file.")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
