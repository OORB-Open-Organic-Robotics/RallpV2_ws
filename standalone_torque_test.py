#!/usr/bin/env python3
"""
Standalone Torque Vectoring Test Suite
Tests the torque vectoring algorithm logic without ROS2 dependencies
"""

import numpy as np
import matplotlib.pyplot as plt
import time
from dataclasses import dataclass
from typing import List, Tuple

@dataclass
class WheelState:
    """Simulated wheel state"""
    velocity: float = 0.0
    torque: float = 0.0
    slip: float = 0.0
    terrain_friction: float = 1.0
    power_consumption: float = 0.0

@dataclass
class RobotCmd:
    """Robot command"""
    linear_x: float = 0.0
    angular_z: float = 0.0

class StandaloneTorqueVectoring:
    """Standalone implementation of torque vectoring algorithm"""
    
    def __init__(self):
        self.wheel_base = 0.3  # meters
        self.track_width = 0.25  # meters
        self.max_torque = 100.0  # Nm
        self.slip_threshold = 0.15
        self.terrain_adaptation = True
        
        # Simulation parameters
        self.dt = 0.01  # 100Hz update rate
        self.wheel_states = [WheelState() for _ in range(4)]  # FL, FR, RL, RR
        
        # Logging
        self.log_data = []
        
    def detect_wheel_slip(self, wheel_idx: int) -> float:
        """Detect wheel slip based on velocity and expected velocity"""
        wheel = self.wheel_states[wheel_idx]
        
        # Simulate slip calculation (in real system, this would use encoders)
        expected_velocity = wheel.torque * wheel.terrain_friction * 0.1
        actual_velocity = wheel.velocity
        
        if expected_velocity > 0.01:
            slip = abs(expected_velocity - actual_velocity) / expected_velocity
        else:
            slip = 0.0
            
        wheel.slip = slip
        return slip
    
    def calculate_base_torques(self, cmd: RobotCmd) -> List[float]:
        """Calculate base torques for differential drive"""
        # Basic differential drive calculation
        left_speed = cmd.linear_x - (cmd.angular_z * self.track_width / 2.0)
        right_speed = cmd.linear_x + (cmd.angular_z * self.track_width / 2.0)
        
        # Convert speeds to torques (simplified model)
        torque_factor = 50.0
        left_torque = left_speed * torque_factor
        right_torque = right_speed * torque_factor
        
        # Distribute to wheels: [FL, FR, RL, RR]
        return [left_torque, right_torque, left_torque, right_torque]
    
    def redistribute_torque(self, base_torques: List[float]) -> List[float]:
        """Apply torque vectoring based on slip and terrain"""
        redistributed = base_torques.copy()
        
        # Detect slip for all wheels
        slips = [self.detect_wheel_slip(i) for i in range(4)]
        
        # Find wheels with excessive slip
        slipping_wheels = [i for i, slip in enumerate(slips) if slip > self.slip_threshold]
        
        if slipping_wheels:
            print(f"Slip detected on wheels: {slipping_wheels}")
            
            # Reduce torque on slipping wheels and redistribute
            for wheel_idx in slipping_wheels:
                reduction_factor = min(0.5, slips[wheel_idx])
                torque_reduction = redistributed[wheel_idx] * reduction_factor
                redistributed[wheel_idx] -= torque_reduction
                
                # Redistribute to non-slipping wheels
                non_slipping = [i for i in range(4) if i not in slipping_wheels]
                if non_slipping:
                    redistribution_per_wheel = torque_reduction / len(non_slipping)
                    for idx in non_slipping:
                        redistributed[idx] += redistribution_per_wheel
        
        # Apply terrain adaptation
        if self.terrain_adaptation:
            for i, torque in enumerate(redistributed):
                friction_factor = self.wheel_states[i].terrain_friction
                redistributed[i] = torque * friction_factor
        
        # Clamp torques to limits
        redistributed = [max(-self.max_torque, min(self.max_torque, t)) for t in redistributed]
        
        return redistributed
    
    def simulate_wheel_dynamics(self, torques: List[float], terrain_conditions: List[float]):
        """Simulate wheel response to applied torques"""
        for i, torque in enumerate(torques):
            wheel = self.wheel_states[i]
            
            # Update terrain friction
            wheel.terrain_friction = terrain_conditions[i]
            
            # Simple wheel dynamics simulation
            wheel.torque = torque
            
            # Velocity response (simplified physics)
            acceleration = torque * wheel.terrain_friction * 0.01
            wheel.velocity += acceleration * self.dt
            
            # Add some damping
            wheel.velocity *= 0.95
            
            # Power consumption (simplified model)
            wheel.power_consumption = abs(torque * wheel.velocity) * 0.001
    
    def update(self, cmd: RobotCmd, terrain_conditions: List[float] = None) -> dict:
        """Main update cycle"""
        if terrain_conditions is None:
            terrain_conditions = [1.0, 1.0, 1.0, 1.0]  # Good traction
        
        # Calculate base torques
        base_torques = self.calculate_base_torques(cmd)
        
        # Apply torque vectoring
        final_torques = self.redistribute_torque(base_torques)
        
        # Simulate wheel response
        self.simulate_wheel_dynamics(final_torques, terrain_conditions)
        
        # Calculate diagnostics
        total_power = sum(w.power_consumption for w in self.wheel_states)
        max_slip = max(w.slip for w in self.wheel_states)
        
        # Log data
        log_entry = {
            'time': time.time(),
            'cmd_linear': cmd.linear_x,
            'cmd_angular': cmd.angular_z,
            'base_torques': base_torques,
            'final_torques': final_torques,
            'wheel_slips': [w.slip for w in self.wheel_states],
            'wheel_velocities': [w.velocity for w in self.wheel_states],
            'power_consumption': total_power,
            'max_slip': max_slip,
            'terrain_conditions': terrain_conditions
        }
        self.log_data.append(log_entry)
        
        return {
            'torques': final_torques,
            'slips': [w.slip for w in self.wheel_states],
            'power': total_power,
            'status': 'ACTIVE' if any(t != 0 for t in final_torques) else 'IDLE'
        }

def run_test_scenarios():
    """Run comprehensive test scenarios"""
    tv = StandaloneTorqueVectoring()
    
    print("=== Torque Vectoring Test Suite ===\n")
    
    # Test 1: Straight line movement
    print("Test 1: Straight line movement")
    cmd = RobotCmd(linear_x=1.0, angular_z=0.0)
    terrain = [1.0, 1.0, 1.0, 1.0]  # Good traction all around
    
    for i in range(10):
        result = tv.update(cmd, terrain)
        if i == 9:  # Print final result
            print(f"  Final torques: {[f'{t:.2f}' for t in result['torques']]}")
            print(f"  Max slip: {result['slips'][np.argmax(result['slips'])]:.4f}")
    
    # Test 2: Turning movement
    print("\nTest 2: Turning movement")
    cmd = RobotCmd(linear_x=0.8, angular_z=1.0)
    
    for i in range(10):
        result = tv.update(cmd, terrain)
        if i == 9:
            print(f"  Final torques: {[f'{t:.2f}' for t in result['torques']]}")
            print(f"  Power consumption: {result['power']:.2f}W")
    
    # Test 3: Low traction scenario
    print("\nTest 3: Low traction on right wheels")
    cmd = RobotCmd(linear_x=1.0, angular_z=0.0)
    terrain = [1.0, 0.3, 1.0, 0.3]  # Low traction on right side
    
    for i in range(20):  # Run longer to see adaptation
        result = tv.update(cmd, terrain)
        if i == 19:
            print(f"  Final torques: {[f'{t:.2f}' for t in result['torques']]}")
            print(f"  Wheel slips: {[f'{s:.4f}' for s in result['slips']]}")
            print(f"  Status: {result['status']}")
    
    # Test 4: Extreme slip scenario
    print("\nTest 4: Extreme slip scenario (ice/mud)")
    cmd = RobotCmd(linear_x=1.2, angular_z=0.5)
    terrain = [0.1, 0.8, 0.2, 0.7]  # Mixed terrain
    
    for i in range(30):
        result = tv.update(cmd, terrain)
        if i == 29:
            print(f"  Final torques: {[f'{t:.2f}' for t in result['torques']]}")
            print(f"  Max slip: {max(result['slips']):.4f}")
            print(f"  Torque vectoring active: {'YES' if max(result['slips']) > tv.slip_threshold else 'NO'}")
    
    # Generate performance plot
    generate_performance_plots(tv.log_data)
    
    print("\n=== Test Complete ===")
    print(f"Total samples logged: {len(tv.log_data)}")
    print("Performance plots saved as 'torque_vectoring_analysis.png'")

def generate_performance_plots(log_data):
    """Generate analysis plots"""
    if not log_data:
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Torque Vectoring Performance Analysis', fontsize=16)
    
    # Extract data
    times = [entry['time'] - log_data[0]['time'] for entry in log_data]
    torques = np.array([entry['final_torques'] for entry in log_data])
    slips = np.array([entry['wheel_slips'] for entry in log_data])
    power = [entry['power_consumption'] for entry in log_data]
    cmds_linear = [entry['cmd_linear'] for entry in log_data]
    cmds_angular = [entry['cmd_angular'] for entry in log_data]
    
    # Plot 1: Torque distribution
    axes[0, 0].plot(times, torques[:, 0], label='FL', linewidth=2)
    axes[0, 0].plot(times, torques[:, 1], label='FR', linewidth=2)
    axes[0, 0].plot(times, torques[:, 2], label='RL', linewidth=2)
    axes[0, 0].plot(times, torques[:, 3], label='RR', linewidth=2)
    axes[0, 0].set_title('Wheel Torque Distribution')
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Torque (Nm)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Plot 2: Slip detection
    axes[0, 1].plot(times, slips[:, 0], label='FL', linewidth=2)
    axes[0, 1].plot(times, slips[:, 1], label='FR', linewidth=2)
    axes[0, 1].plot(times, slips[:, 2], label='RL', linewidth=2)
    axes[0, 1].plot(times, slips[:, 3], label='RR', linewidth=2)
    axes[0, 1].axhline(y=0.15, color='r', linestyle='--', label='Slip Threshold')
    axes[0, 1].set_title('Wheel Slip Detection')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Slip Ratio')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Plot 3: Power consumption
    axes[1, 0].plot(times, power, 'g-', linewidth=2, label='Total Power')
    axes[1, 0].set_title('Power Consumption')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Power (W)')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Plot 4: Command vs Response
    axes[1, 1].plot(times, cmds_linear, 'b-', linewidth=2, label='Linear Cmd')
    axes[1, 1].plot(times, cmds_angular, 'r-', linewidth=2, label='Angular Cmd')
    axes[1, 1].set_title('Command Inputs')
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Command Value')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('torque_vectoring_analysis.png', dpi=300, bbox_inches='tight')
    print("Performance analysis plot saved!")

if __name__ == "__main__":
    run_test_scenarios()
