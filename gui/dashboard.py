"""
GUI Dashboard Module

Implements Tkinter-based graphical user interface for real-time monitoring
and control of the aerospace-grade pressure regulation system.

Features:
- Real-time pressure, valve angle, and motor current displays
- Live matplotlib plots
- Setpoint control
- Start/Stop/Reset buttons
- Safety status indicators
- Performance metrics display

Author: Industrial Pressure Control System
Date: 2026-02-18
"""

import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import logging

logger = logging.getLogger(__name__)


class PressureControlDashboard:
    """
    Main GUI dashboard for pressure control system.
    """
    
    def __init__(self, root, simulator):
        """
        Initialize the dashboard.
        
        Args:
            root: Tkinter root window
            simulator: PressureControlSimulator instance
        """
        self.root = root
        self.simulator = simulator
        
        # GUI state
        self.is_running = False
        self.update_interval = 50  # ms
        
        # Create all widgets
        self.create_widgets()
        
        logger.info("Dashboard initialized")
    
    def create_widgets(self):
        """Create all GUI widgets."""
        # Main container
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Create panels
        self.create_control_panel(main_frame)
        self.create_display_panel(main_frame)
        self.create_plot_panel(main_frame)
        self.create_safety_panel(main_frame)
    
    def create_control_panel(self, parent):
        """Create control panel with setpoint input and buttons."""
        control_frame = ttk.LabelFrame(parent, text="Control Panel", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N), padx=5, pady=5)
        
        # Setpoint control
        ttk.Label(control_frame, text="Setpoint (bar):").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.setpoint_var = tk.StringVar(value="350")
        setpoint_entry = ttk.Entry(control_frame, textvariable=self.setpoint_var, width=10)
        setpoint_entry.grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(control_frame, text="Set", command=self.set_setpoint).grid(row=0, column=2, padx=5, pady=5)
        
        # Control buttons
        self.start_button = ttk.Button(control_frame, text="Start", command=self.start_simulation)
        self.start_button.grid(row=1, column=0, padx=5, pady=5)
        
        self.stop_button = ttk.Button(control_frame, text="Stop", command=self.stop_simulation, state=tk.DISABLED)
        self.stop_button.grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Button(control_frame, text="Reset", command=self.reset_simulation).grid(row=1, column=2, padx=5, pady=5)
    
    def create_display_panel(self, parent):
        """Create display panel for real-time values."""
        display_frame = ttk.LabelFrame(parent, text="Real-Time Measurements", padding="10")
        display_frame.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N), padx=5, pady=5)
        
        # Pressure display
        ttk.Label(display_frame, text="Pressure:", font=('Arial', 10, 'bold')).grid(row=0, column=0, sticky=tk.W, pady=5)
        self.pressure_label = ttk.Label(display_frame, text="0.0 bar", font=('Arial', 14))
        self.pressure_label.grid(row=0, column=1, sticky=tk.W, padx=10, pady=5)
        
        # Valve angle display
        ttk.Label(display_frame, text="Valve Angle:", font=('Arial', 10, 'bold')).grid(row=1, column=0, sticky=tk.W, pady=5)
        self.valve_label = ttk.Label(display_frame, text="0.0°", font=('Arial', 14))
        self.valve_label.grid(row=1, column=1, sticky=tk.W, padx=10, pady=5)
        
        # Motor current display
        ttk.Label(display_frame, text="Motor Current:", font=('Arial', 10, 'bold')).grid(row=2, column=0, sticky=tk.W, pady=5)
        self.current_label = ttk.Label(display_frame, text="0.0 A", font=('Arial', 14))
        self.current_label.grid(row=2, column=1, sticky=tk.W, padx=10, pady=5)
        
        # Control voltage display
        ttk.Label(display_frame, text="Control Voltage:", font=('Arial', 10, 'bold')).grid(row=3, column=0, sticky=tk.W, pady=5)
        self.voltage_label = ttk.Label(display_frame, text="0.0 V", font=('Arial', 14))
        self.voltage_label.grid(row=3, column=1, sticky=tk.W, padx=10, pady=5)
        
        # Time display
        ttk.Label(display_frame, text="Time:", font=('Arial', 10, 'bold')).grid(row=4, column=0, sticky=tk.W, pady=5)
        self.time_label = ttk.Label(display_frame, text="0.0 s", font=('Arial', 14))
        self.time_label.grid(row=4, column=1, sticky=tk.W, padx=10, pady=5)
    
    def create_safety_panel(self, parent):
        """Create safety status panel."""
        safety_frame = ttk.LabelFrame(parent, text="Safety Status", padding="10")
        safety_frame.grid(row=0, column=2, sticky=(tk.W, tk.E, tk.N), padx=5, pady=5)
        
        # Emergency stop indicator
        ttk.Label(safety_frame, text="System Status:", font=('Arial', 10, 'bold')).grid(row=0, column=0, sticky=tk.W, pady=5)
        self.status_label = ttk.Label(safety_frame, text="READY", font=('Arial', 12), foreground='green')
        self.status_label.grid(row=0, column=1, sticky=tk.W, padx=10, pady=5)
    
    def create_plot_panel(self, parent):
        """Create matplotlib plot panel for live graphing."""
        plot_frame = ttk.LabelFrame(parent, text="Live Plots", padding="10")
        plot_frame.grid(row=1, column=0, columnspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), padx=5, pady=5)
        
        # Create figure with subplots
        self.fig = Figure(figsize=(12, 6), dpi=100)
        
        # Pressure plot
        self.ax1 = self.fig.add_subplot(2, 2, 1)
        self.ax1.set_title('Pressure vs Time')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Pressure (bar)')
        self.ax1.grid(True, alpha=0.3)
        self.pressure_line, = self.ax1.plot([], [], 'b-', label='Pressure')
        self.setpoint_line, = self.ax1.plot([], [], 'r--', label='Setpoint')
        self.ax1.legend()
        
        # Valve angle plot
        self.ax2 = self.fig.add_subplot(2, 2, 2)
        self.ax2.set_title('Valve Angle vs Time')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Angle (deg)')
        self.ax2.grid(True, alpha=0.3)
        self.valve_line, = self.ax2.plot([], [], 'g-')
        
        # Motor current plot
        self.ax3 = self.fig.add_subplot(2, 2, 3)
        self.ax3.set_title('Motor Current vs Time')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Current (A)')
        self.ax3.grid(True, alpha=0.3)
        self.current_line, = self.ax3.plot([], [], 'm-')
        
        # Control voltage plot
        self.ax4 = self.fig.add_subplot(2, 2, 4)
        self.ax4.set_title('Control Voltage vs Time')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('Voltage (V)')
        self.ax4.grid(True, alpha=0.3)
        self.voltage_line, = self.ax4.plot([], [], 'c-')
        
        self.fig.tight_layout()
        
        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def start_simulation(self):
        """Start the simulation."""
        if not self.is_running:
            self.is_running = True
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
            self.status_label.config(text="RUNNING", foreground='blue')
            
            # Initialize simulator if needed
            if self.simulator.time == 0.0:
                setpoint = float(self.setpoint_var.get())
                self.simulator.initialize(initial_pressure=0.0, initial_setpoint=setpoint)
            
            # Start update loop
            self.update_loop()
            logger.info("Simulation started")
    
    def stop_simulation(self):
        """Stop the simulation."""
        self.is_running = False
        self.start_button.config(state=tk.NORMAL)
        self.stop_button.config(state=tk.DISABLED)
        self.status_label.config(text="STOPPED", foreground='orange')
        logger.info("Simulation stopped")
    
    def reset_simulation(self):
        """Reset the simulation."""
        self.stop_simulation()
        setpoint = float(self.setpoint_var.get())
        self.simulator.initialize(initial_pressure=0.0, initial_setpoint=setpoint)
        self.update_displays()
        self.update_plots()
        self.status_label.config(text="READY", foreground='green')
        logger.info("Simulation reset")
    
    def set_setpoint(self):
        """Set new pressure setpoint from user input."""
        try:
            setpoint = float(self.setpoint_var.get())
            self.simulator.controller.set_setpoint(setpoint)
            logger.info(f"Setpoint updated to {setpoint} bar")
        except ValueError:
            logger.error("Invalid setpoint value")
    
    def update_loop(self):
        """Main update loop for simulation and GUI."""
        if self.is_running:
            # Step simulation
            dt = self.update_interval / 1000.0  # Convert ms to seconds
            state = self.simulator.step(dt)
            
            # Update displays
            self.update_displays()
            self.update_plots()
            
            # Check for emergency stop
            if state['emergency_stopped']:
                self.status_label.config(text="EMERGENCY STOP", foreground='red')
                self.stop_simulation()
                return
            
            # Schedule next update
            self.root.after(self.update_interval, self.update_loop)
    
    def update_displays(self):
        """Update real-time display values."""
        state = self.simulator.get_current_state()
        
        self.pressure_label.config(text=f"{state['pressure']:.2f} bar")
        self.valve_label.config(text=f"{state['valve_angle_deg']:.2f}°")
        self.current_label.config(text=f"{state['motor_current']:.2f} A")
        self.voltage_label.config(text=f"{state['control_voltage']:.2f} V")
        self.time_label.config(text=f"{state['time']:.2f} s")
    
    def update_plots(self):
        """Update the live plots."""
        history = self.simulator.get_history()
        
        if len(history['time']) > 1:
            # Update pressure plot
            self.pressure_line.set_data(history['time'], history['pressure'])
            self.setpoint_line.set_data(history['time'], history['setpoint'])
            self.ax1.relim()
            self.ax1.autoscale_view()
            
            # Update valve plot
            self.valve_line.set_data(history['time'], history['valve_angle_deg'])
            self.ax2.relim()
            self.ax2.autoscale_view()
            
            # Update current plot
            self.current_line.set_data(history['time'], history['motor_current'])
            self.ax3.relim()
            self.ax3.autoscale_view()
            
            # Update voltage plot
            self.voltage_line.set_data(history['time'], history['control_voltage'])
            self.ax4.relim()
            self.ax4.autoscale_view()
            
            # Redraw canvas
            self.canvas.draw()
