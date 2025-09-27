#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from diagnostic_msgs.msg import DiagnosticArray
import matplotlib.pyplot as plt
import numpy as np
from collections import deque
import threading
import time

class TorqueVectoringVisualizer(Node):
    def __init__(self):
        super().__init__('torque_vectoring_visualizer')
        
        # Data storage
        self.max_points = 100
        self.time_data = deque(maxlen=self.max_points)
        self.torque_data = [deque(maxlen=self.max_points) for _ in range(4)]
        self.slip_data = [deque(maxlen=self.max_points) for _ in range(4)]
        self.power_data = deque(maxlen=self.max_points)
        
        self.wheel_names = ['FL', 'FR', 'RL', 'RR']
        self.start_time = time.time()
        
        # Subscribers
        self.debug_sub = self.create_subscription(
            Float64MultiArray, '/vectoring/debug', self.debug_callback, 10)
        
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)
        
        # Setup matplotlib
        plt.ion()
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('Torque Vectoring System Monitor')
        
        # Configure plots
        self.setup_plots()
        
        # Start plotting thread
        self.plot_thread = threading.Thread(target=self.update_plots, daemon=True)
        self.plot_thread.start()
        
        self.get_logger().info("Torque Vectoring Visualizer started")

    def setup_plots(self):
        # Torque plot
        self.axes[0].set_title('Wheel Torques')
        self.axes[0].set_ylabel('Torque (Nm)')
        self.axes[0].grid(True)
        self.axes[0].legend(self.wheel_names)
        
        # Slip ratio plot
        self.axes[1].set_title('Slip Ratios')
        self.axes[1].set_ylabel('Slip Ratio')
        self.axes[1].grid(True)
        self.axes[1].legend(self.wheel_names)
        self.axes[1].axhline(y=0.2, color='r', linestyle='--', alpha=0.5, label='Slip Threshold')
        self.axes[1].axhline(y=-0.2, color='r', linestyle='--', alpha=0.5)
        
        # Power consumption plot
        self.axes[2].set_title('Power Consumption')
        self.axes[2].set_ylabel('Power (W)')
        self.axes[2].set_xlabel('Time (s)')
        self.axes[2].grid(True)

    def debug_callback(self, msg):
        if len(msg.data) >= 16:
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)
            
            # Extract data: [wheel_torques(4), slip_ratios(4), wheel_velocities(4), power, angular_vel, linear_vel, sensors_healthy]
            for i in range(4):
                self.torque_data[i].append(msg.data[i])
                self.slip_data[i].append(msg.data[i + 4])
            
            self.power_data.append(msg.data[12])

    def diagnostics_callback(self, msg):
        for status in msg.status:
            if status.name == "torque_vectoring_system":
                # Extract additional diagnostic information
                for kv in status.values:
                    if kv.key == "terrain_type":
                        # Could display terrain type on plot
                        pass

    def update_plots(self):
        while rclpy.ok():
            try:
                if len(self.time_data) > 1:
                    # Clear previous plots
                    for ax in self.axes:
                        ax.clear()
                    
                    self.setup_plots()
                    
                    time_array = np.array(self.time_data)
                    
                    # Plot torques
                    colors = ['blue', 'red', 'green', 'orange']
                    for i in range(4):
                        if len(self.torque_data[i]) > 0:
                            self.axes[0].plot(time_array, list(self.torque_data[i]), 
                                            color=colors[i], label=self.wheel_names[i])
                    
                    # Plot slip ratios
                    for i in range(4):
                        if len(self.slip_data[i]) > 0:
                            self.axes[1].plot(time_array, list(self.slip_data[i]), 
                                            color=colors[i], label=self.wheel_names[i])
                    
                    # Add slip threshold lines
                    self.axes[1].axhline(y=0.2, color='r', linestyle='--', alpha=0.5)
                    self.axes[1].axhline(y=-0.2, color='r', linestyle='--', alpha=0.5)
                    
                    # Plot power
                    if len(self.power_data) > 0:
                        self.axes[2].plot(time_array, list(self.power_data), 'purple', linewidth=2)
                    
                    # Update legends and labels
                    for ax in self.axes:
                        ax.legend()
                        ax.grid(True)
                    
                    plt.tight_layout()
                    plt.pause(0.1)
                
                time.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f"Plotting error: {e}")
                time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TorqueVectoringVisualizer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        plt.close('all')
        rclpy.shutdown()


if __name__ == '__main__':
    main()
