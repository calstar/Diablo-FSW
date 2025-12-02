#!/usr/bin/env python3
"""
Plot motor simulation data: target and actual position vs time
Run this script after running motor_simulation.cpp to visualize the results.
"""

import matplotlib.pyplot as plt
import pandas as pd
import sys
import os

def plot_motor_data(csv_file='motor_simulation_data.csv'):
    """Plot target and actual position vs time from CSV data."""
    
    # Check if file exists
    if not os.path.exists(csv_file):
        print(f"Error: File '{csv_file}' not found.")
        print("Please run motor_simulation.cpp first to generate the data.")
        return
    
    # Read CSV data
    try:
        data = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV file: {e}")
        return
    
    # Create figure with subplots
    fig, axes = plt.subplots(3, 1, figsize=(10, 8))
    fig.suptitle('Motor Control Simulation Results', fontsize=14, fontweight='bold')
    
    # Plot 1: Position vs Time
    axes[0].plot(data['time'], data['target_position'], 'r--', label='Target Position', linewidth=2)
    axes[0].plot(data['time'], data['actual_position'], 'b-', label='Actual Position', linewidth=1.5)
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Position (rad)')
    axes[0].set_title('Position vs Time')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # Plot 2: Velocity vs Time
    axes[1].plot(data['time'], data['target_velocity'], 'r--', label='Target Velocity', linewidth=2)
    axes[1].plot(data['time'], data['actual_velocity'], 'b-', label='Actual Velocity', linewidth=1.5)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Velocity (rad/s)')
    axes[1].set_title('Velocity vs Time')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)
    
    # Plot 3: Control Voltage vs Time
    axes[2].plot(data['time'], data['control_voltage'], 'g-', label='Control Voltage', linewidth=1.5)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Voltage (V)')
    axes[2].set_title('Control Voltage vs Time')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save plot
    output_file = 'motor_simulation_plot.png'
    plt.savefig(output_file, dpi=150)
    print(f"Plot saved to {output_file}")
    
    # Show plot
    plt.show()

if __name__ == '__main__':
    # Allow CSV file to be specified as command line argument
    csv_file = sys.argv[1] if len(sys.argv) > 1 else 'motor_simulation_data.csv'
    plot_motor_data(csv_file)

