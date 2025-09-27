#!/usr/bin/env python3
"""
Signal-Level Test Framework for Torque Vectoring System

This framework provides comprehensive signal injection and validation 
capabilities for testing the torque vectoring algorithms without 
requiring physical hardware.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, Header
from diagnostic_msgs.msg import DiagnosticArray
import numpy as np
import time
import math
import json
import threading
from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from collections import deque

@dataclass
class TestSignal:
    """Represents a test signal with timing and data"""
    timestamp: float
    data: Any
    signal_type: str
    
@dataclass
class TestResult:
    """Test execution result"""
    test_name: str
    passed: bool
    execution_time: float
    details: Dict[str, Any]
    error_message: Optional[str] = None

class SignalGenerator:
    """Generates synthetic sensor signals for testing"""
    
    def __init__(self):
        self.time_offset = 0.0
        
    def generate_imu_signal(self, linear_accel: tuple, angular_vel: tuple, 
                          noise_level: float = 0.0) -> Imu:
        """Generate IMU signal with optional noise"""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp.sec = int(time.time())
        msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
        msg.header.frame_id = "base_link"
        
        # Add noise if specified
        if noise_level > 0:
            accel_noise = np.random.normal(0, noise_level, 3)
            angular_noise = np.random.normal(0, noise_level * 0.5, 3)
        else:
            accel_noise = [0, 0, 0]
            angular_noise = [0, 0, 0]
        
        msg.linear_acceleration.x = linear_accel[0] + accel_noise[0]
        msg.linear_acceleration.y = linear_accel[1] + accel_noise[1]
        msg.linear_acceleration.z = linear_accel[2] + accel_noise[2]
        
        msg.angular_velocity.x = angular_vel[0] + angular_noise[0]
        msg.angular_velocity.y = angular_vel[1] + angular_noise[1]
        msg.angular_velocity.z = angular_vel[2] + angular_noise[2]
        
        return msg
    
    def generate_joint_states(self, wheel_positions: List[float], 
                            wheel_velocities: List[float],
                            noise_level: float = 0.0) -> JointState:
        """Generate joint state signals with optional noise"""
        msg = JointState()
        msg.header = Header()
        msg.header.stamp.sec = int(time.time())
        msg.header.stamp.nanosec = int((time.time() % 1) * 1e9)
        
        wheel_names = ['fl_joint', 'fr_joint', 'rl_joint', 'rr_joint']
        msg.name = wheel_names
        
        # Add noise if specified
        if noise_level > 0:
            pos_noise = np.random.normal(0, noise_level * 0.01, 4)
            vel_noise = np.random.normal(0, noise_level * 0.1, 4)
        else:
            pos_noise = [0, 0, 0, 0]
            vel_noise = [0, 0, 0, 0]
        
        msg.position = [pos + noise for pos, noise in zip(wheel_positions, pos_noise)]
        msg.velocity = [vel + noise for vel, noise in zip(wheel_velocities, vel_noise)]
        msg.effort = [0.0] * 4  # Not used in current implementation
        
        return msg
    
    def generate_cmd_vel(self, linear_x: float, angular_z: float) -> Twist:
        """Generate command velocity signal"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = angular_z
        return msg

class TerrainSimulator:
    """Simulates different terrain conditions"""
    
    @staticmethod
    def normal_terrain():
        """Normal terrain with minimal slip"""
        return {
            'slip_coefficient': 0.05,
            'resistance_factor': 1.0,
            'noise_level': 0.02
        }
    
    @staticmethod
    def slippery_terrain():
        """Slippery terrain (ice, wet surfaces)"""
        return {
            'slip_coefficient': 0.4,
            'resistance_factor': 0.8,
            'noise_level': 0.05
        }
    
    @staticmethod
    def high_resistance_terrain():
        """High resistance terrain (sand, gravel)"""
        return {
            'slip_coefficient': 0.15,
            'resistance_factor': 1.5,
            'noise_level': 0.03
        }
    
    @staticmethod
    def mixed_terrain():
        """Mixed terrain with different conditions per wheel"""
        return {
            'wheel_conditions': [
                {'slip_coefficient': 0.05, 'resistance_factor': 1.0},  # FL - normal
                {'slip_coefficient': 0.3, 'resistance_factor': 0.9},   # FR - slippery
                {'slip_coefficient': 0.1, 'resistance_factor': 1.2},   # RL - slightly rough
                {'slip_coefficient': 0.2, 'resistance_factor': 1.1}    # RR - moderate slip
            ],
            'noise_level': 0.04
        }

class SignalLevelTestFramework(Node):
    """Main test framework for signal-level testing"""
    
    def __init__(self):
        super().__init__('signal_level_test_framework')
        
        # Publishers for signal injection
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        
        # Subscribers for monitoring outputs
        self.debug_sub = self.create_subscription(
            Float64MultiArray, '/vectoring/debug', self.debug_callback, 10)
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)
        self.output_cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.output_cmd_vel_callback, 10)
        
        # Test data storage
        self.debug_data_history = deque(maxlen=100)
        self.diagnostics_history = deque(maxlen=50)
        self.output_cmd_vel_history = deque(maxlen=100)
        
        # Test execution state
        self.current_test = None
        self.test_results = []
        self.test_start_time = 0.0
        
        # Signal generator and terrain simulator
        self.signal_gen = SignalGenerator()
        self.terrain_sim = TerrainSimulator()
        
        # Robot parameters (should match torque_vectoring_node parameters)
        self.wheel_radius = 0.125
        self.wheel_separation = 0.59
        self.max_torque = 10.0
        
        self.get_logger().info("Signal-Level Test Framework initialized")
    
    def debug_callback(self, msg):
        """Store debug data for analysis"""
        timestamp = time.time()
        self.debug_data_history.append({
            'timestamp': timestamp,
            'data': msg.data
        })
    
    def diagnostics_callback(self, msg):
        """Store diagnostics data for analysis"""
        timestamp = time.time()
        self.diagnostics_history.append({
            'timestamp': timestamp,
            'data': msg
        })
    
    def output_cmd_vel_callback(self, msg):
        """Store output cmd_vel for analysis"""
        timestamp = time.time()
        self.output_cmd_vel_history.append({
            'timestamp': timestamp,
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        })
    
    def inject_signal_sequence(self, signals: List[TestSignal], duration: float):
        """Inject a sequence of signals over a specified duration"""
        start_time = time.time()
        signal_index = 0
        
        while time.time() - start_time < duration and signal_index < len(signals):
            current_time = time.time() - start_time
            signal = signals[signal_index]
            
            if current_time >= signal.timestamp:
                if signal.signal_type == 'imu':
                    self.imu_pub.publish(signal.data)
                elif signal.signal_type == 'joint_states':
                    self.joint_states_pub.publish(signal.data)
                elif signal.signal_type == 'cmd_vel':
                    self.cmd_vel_pub.publish(signal.data)
                
                signal_index += 1
            
            time.sleep(0.01)  # 100Hz injection rate
    
    def validate_torque_distribution(self, expected_behavior: str) -> bool:
        """Validate torque distribution behavior"""
        if not self.debug_data_history:
            return False
        
        latest_data = self.debug_data_history[-1]['data']
        if len(latest_data) < 4:
            return False
        
        torques = latest_data[0:4]  # FL, FR, RL, RR
        
        if expected_behavior == 'equal_distribution':
            # Check if torques are approximately equal
            torque_std = np.std(torques)
            return torque_std < 0.5  # Allow some variation
        
        elif expected_behavior == 'left_turn_boost':
            # Right wheels should have higher torque for left turn
            right_avg = (torques[1] + torques[3]) / 2  # FR + RR
            left_avg = (torques[0] + torques[2]) / 2   # FL + RL
            return right_avg > left_avg + 0.1
        
        elif expected_behavior == 'right_turn_boost':
            # Left wheels should have higher torque for right turn
            left_avg = (torques[0] + torques[2]) / 2   # FL + RL
            right_avg = (torques[1] + torques[3]) / 2  # FR + RR
            return left_avg > right_avg + 0.1
        
        elif expected_behavior == 'slip_compensation':
            # Check if any wheel has reduced torque due to slip
            if len(latest_data) >= 8:
                slip_ratios = latest_data[4:8]
                for i, slip in enumerate(slip_ratios):
                    if abs(slip) > 0.2:  # Significant slip
                        # Corresponding torque should be reduced
                        if abs(torques[i]) > self.max_torque * 0.8:
                            return False
                return True
        
        return False
    
    def validate_power_consumption(self, max_power: float) -> bool:
        """Validate power consumption is within limits"""
        if not self.debug_data_history:
            return False
        
        latest_data = self.debug_data_history[-1]['data']
        if len(latest_data) < 13:
            return False
        
        power_consumption = latest_data[12]
        return power_consumption <= max_power
    
    def validate_slip_detection(self, expected_slip_wheels: List[int]) -> bool:
        """Validate slip detection for specific wheels"""
        if not self.debug_data_history:
            return False
        
        latest_data = self.debug_data_history[-1]['data']
        if len(latest_data) < 8:
            return False
        
        slip_ratios = latest_data[4:8]
        
        for wheel_idx in expected_slip_wheels:
            if wheel_idx < 4 and abs(slip_ratios[wheel_idx]) < 0.15:
                return False  # Expected slip but not detected
        
        return True
    
    def run_test(self, test_name: str, test_function, *args, **kwargs) -> TestResult:
        """Execute a test and record results"""
        self.get_logger().info(f"Starting test: {test_name}")
        start_time = time.time()
        
        try:
            # Clear previous data
            self.debug_data_history.clear()
            self.diagnostics_history.clear()
            self.output_cmd_vel_history.clear()
            
            # Execute test
            result = test_function(*args, **kwargs)
            
            execution_time = time.time() - start_time
            
            test_result = TestResult(
                test_name=test_name,
                passed=result['passed'],
                execution_time=execution_time,
                details=result.get('details', {}),
                error_message=result.get('error', None)
            )
            
            self.test_results.append(test_result)
            self.get_logger().info(
                f"Test {test_name}: {'PASSED' if result['passed'] else 'FAILED'} "
                f"({execution_time:.2f}s)")
            
            return test_result
            
        except Exception as e:
            execution_time = time.time() - start_time
            error_msg = f"Test execution failed: {str(e)}"
            
            test_result = TestResult(
                test_name=test_name,
                passed=False,
                execution_time=execution_time,
                details={},
                error_message=error_msg
            )
            
            self.test_results.append(test_result)
            self.get_logger().error(f"Test {test_name} FAILED: {error_msg}")
            
            return test_result
    
    def generate_test_report(self) -> Dict[str, Any]:
        """Generate comprehensive test report"""
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results if result.passed)
        failed_tests = total_tests - passed_tests
        
        total_execution_time = sum(result.execution_time for result in self.test_results)
        
        report = {
            'summary': {
                'total_tests': total_tests,
                'passed': passed_tests,
                'failed': failed_tests,
                'success_rate': (passed_tests / total_tests * 100) if total_tests > 0 else 0,
                'total_execution_time': total_execution_time
            },
            'test_results': [
                {
                    'name': result.test_name,
                    'passed': result.passed,
                    'execution_time': result.execution_time,
                    'details': result.details,
                    'error_message': result.error_message
                }
                for result in self.test_results
            ]
        }
        
        return report
    
    def save_test_report(self, filename: str):
        """Save test report to JSON file"""
        report = self.generate_test_report()
        
        with open(filename, 'w') as f:
            json.dump(report, f, indent=2)
        
        self.get_logger().info(f"Test report saved to {filename}")

def main(args=None):
    rclpy.init(args=args)
    framework = SignalLevelTestFramework()
    
    try:
        rclpy.spin(framework)
    except KeyboardInterrupt:
        pass
    finally:
        framework.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()