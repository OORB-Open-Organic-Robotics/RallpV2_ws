#!/usr/bin/env python3
"""
Comprehensive Slip Detection Test Suite

This module provides extensive testing for slip detection algorithms
including various terrain conditions, sensor noise, and edge cases.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
from typing import List, Dict, Any, Tuple
from signal_level_test_framework import SignalLevelTestFramework, SignalGenerator, TerrainSimulator, TestSignal

class SlipDetectionTestSuite:
    """Comprehensive test suite for slip detection algorithms"""
    
    def __init__(self, test_framework: SignalLevelTestFramework):
        self.framework = test_framework
        self.signal_gen = SignalGenerator()
        self.terrain_sim = TerrainSimulator()
        
        # Test parameters
        self.wheel_radius = 0.125
        self.wheel_separation = 0.59
        self.test_duration = 3.0  # seconds per test
    
    def test_no_slip_condition(self) -> Dict[str, Any]:
        """Test slip detection when no slip is occurring"""
        
        # Generate signals for straight line motion with no slip
        linear_vel = 1.0  # m/s
        angular_vel = 0.0  # rad/s
        
        # Calculate expected wheel velocities
        expected_wheel_vel = linear_vel / self.wheel_radius
        
        signals = []
        dt = 0.02  # 50Hz
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # IMU signal - consistent linear acceleration
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(0.0, 0.0, 0.0),  # Constant velocity
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.01
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states - wheels moving at expected velocity
            wheel_positions = [expected_wheel_vel * timestamp] * 4
            wheel_velocities = [expected_wheel_vel] * 4
            
            joint_msg = self.signal_gen.generate_joint_states(
                wheel_positions, wheel_velocities, noise_level=0.01
            )
            signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
            
            # Command velocity
            cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
            signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
        
        # Inject signals
        self.framework.inject_signal_sequence(signals, self.test_duration)
        
        # Wait for processing
        time.sleep(1.0)
        
        # Validate no slip detected
        if self.framework.debug_data_history:
            latest_data = self.framework.debug_data_history[-1]['data']
            if len(latest_data) >= 8:
                slip_ratios = latest_data[4:8]
                max_slip = max(abs(slip) for slip in slip_ratios)
                
                passed = max_slip < 0.1  # Should be minimal slip
                return {
                    'passed': passed,
                    'details': {
                        'max_slip_ratio': max_slip,
                        'slip_ratios': slip_ratios,
                        'expected_threshold': 0.1
                    }
                }
        
        return {'passed': False, 'error': 'No debug data received'}
    
    def test_single_wheel_slip(self) -> Dict[str, Any]:
        """Test detection of slip on a single wheel"""
        
        linear_vel = 1.5  # m/s
        angular_vel = 0.0  # rad/s
        slip_wheel_index = 1  # Front right wheel
        slip_factor = 0.4  # 40% slip
        
        # Calculate expected wheel velocities
        expected_wheel_vel = linear_vel / self.wheel_radius
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # IMU signal
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(0.0, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.02
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states with one slipping wheel
            wheel_positions = [expected_wheel_vel * timestamp] * 4
            wheel_velocities = [expected_wheel_vel] * 4
            
            # Introduce slip on specified wheel
            if timestamp > 0.5:  # Start slip after 0.5 seconds
                wheel_velocities[slip_wheel_index] *= (1.0 + slip_factor)
                wheel_positions[slip_wheel_index] = (
                    0.5 * expected_wheel_vel +  # Normal motion for first 0.5s
                    (timestamp - 0.5) * wheel_velocities[slip_wheel_index]  # Slipping motion
                )
            
            joint_msg = self.signal_gen.generate_joint_states(
                wheel_positions, wheel_velocities, noise_level=0.02
            )
            signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
            
            # Command velocity
            cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
            signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
        
        # Inject signals
        self.framework.inject_signal_sequence(signals, self.test_duration)
        
        # Wait for processing
        time.sleep(1.0)
        
        # Validate slip detection
        if self.framework.debug_data_history:
            latest_data = self.framework.debug_data_history[-1]['data']
            if len(latest_data) >= 8:
                slip_ratios = latest_data[4:8]
                slip_detected = abs(slip_ratios[slip_wheel_index]) > 0.2
                other_wheels_ok = all(abs(slip_ratios[i]) < 0.15 for i in range(4) if i != slip_wheel_index)
                
                passed = slip_detected and other_wheels_ok
                return {
                    'passed': passed,
                    'details': {
                        'slip_ratios': slip_ratios,
                        'slipping_wheel': slip_wheel_index,
                        'slip_detected': slip_detected,
                        'other_wheels_ok': other_wheels_ok,
                        'expected_slip_factor': slip_factor
                    }
                }
        
        return {'passed': False, 'error': 'No debug data received'}
    
    def test_turn_induced_slip(self) -> Dict[str, Any]:
        """Test slip detection during turning maneuvers"""
        
        linear_vel = 1.2  # m/s
        angular_vel = 0.8  # rad/s (sharp turn)
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # IMU signal with turning
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(0.0, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.02
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Calculate differential wheel speeds for turning
            left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            
            wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
            wheel_positions = [vel * timestamp for vel in wheel_velocities]
            
            # Add slip to outer wheels during aggressive turning
            if timestamp > 1.0:  # Start slip after 1 second
                slip_factor = 0.3
                wheel_velocities[1] *= (1.0 + slip_factor)  # FR wheel
                wheel_velocities[3] *= (1.0 + slip_factor)  # RR wheel
            
            joint_msg = self.signal_gen.generate_joint_states(
                wheel_positions, wheel_velocities, noise_level=0.03
            )
            signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
            
            # Command velocity
            cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
            signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
        
        # Inject signals
        self.framework.inject_signal_sequence(signals, self.test_duration)
        
        # Wait for processing
        time.sleep(1.0)
        
        # Validate slip detection on outer wheels
        slip_detected = self.framework.validate_slip_detection([1, 3])  # FR and RR wheels
        
        return {
            'passed': slip_detected,
            'details': {
                'turn_direction': 'left',
                'angular_velocity': angular_vel,
                'expected_slip_wheels': [1, 3],
                'slip_detected': slip_detected
            }
        }
    
    def test_terrain_based_slip(self) -> Dict[str, Any]:
        """Test slip detection on different terrain types"""
        
        terrain_conditions = [
            ('normal', self.terrain_sim.normal_terrain()),
            ('slippery', self.terrain_sim.slippery_terrain()),
            ('high_resistance', self.terrain_sim.high_resistance_terrain()),
            ('mixed', self.terrain_sim.mixed_terrain())
        ]
        
        results = {}
        
        for terrain_name, terrain_config in terrain_conditions:
            
            linear_vel = 1.0  # m/s
            angular_vel = 0.0  # rad/s
            
            signals = []
            dt = 0.02
            
            for i in range(int(self.test_duration / dt)):
                timestamp = i * dt
                
                # IMU signal
                noise_level = terrain_config.get('noise_level', 0.02)
                imu_msg = self.signal_gen.generate_imu_signal(
                    linear_accel=(0.0, 0.0, 0.0),
                    angular_vel=(0.0, 0.0, angular_vel),
                    noise_level=noise_level
                )
                signals.append(TestSignal(timestamp, imu_msg, 'imu'))
                
                # Calculate wheel velocities with terrain effects
                expected_wheel_vel = linear_vel / self.wheel_radius
                
                if 'wheel_conditions' in terrain_config:  # Mixed terrain
                    wheel_velocities = []
                    for wheel_idx, wheel_config in enumerate(terrain_config['wheel_conditions']):
                        slip_coeff = wheel_config['slip_coefficient']
                        resistance = wheel_config['resistance_factor']
                        
                        # Apply terrain effects
                        actual_vel = expected_wheel_vel * (1.0 + slip_coeff) / resistance
                        wheel_velocities.append(actual_vel)
                else:  # Uniform terrain
                    slip_coeff = terrain_config['slip_coefficient']
                    resistance = terrain_config['resistance_factor']
                    actual_vel = expected_wheel_vel * (1.0 + slip_coeff) / resistance
                    wheel_velocities = [actual_vel] * 4
                
                wheel_positions = [vel * timestamp for vel in wheel_velocities]
                
                joint_msg = self.signal_gen.generate_joint_states(
                    wheel_positions, wheel_velocities, noise_level=noise_level
                )
                signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
                
                # Command velocity
                cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
                signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
            
            # Inject signals
            self.framework.inject_signal_sequence(signals, self.test_duration)
            
            # Wait for processing
            time.sleep(1.0)
            
            # Analyze results for this terrain
            if self.framework.debug_data_history:
                latest_data = self.framework.debug_data_history[-1]['data']
                if len(latest_data) >= 8:
                    slip_ratios = latest_data[4:8]
                    max_slip = max(abs(slip) for slip in slip_ratios)
                    
                    # Validate slip detection based on terrain type
                    if terrain_name == 'normal':
                        expected_low_slip = max_slip < 0.1
                        results[terrain_name] = {
                            'passed': expected_low_slip,
                            'max_slip': max_slip,
                            'slip_ratios': slip_ratios
                        }
                    elif terrain_name == 'slippery':
                        expected_high_slip = max_slip > 0.25
                        results[terrain_name] = {
                            'passed': expected_high_slip,
                            'max_slip': max_slip,
                            'slip_ratios': slip_ratios
                        }
                    else:
                        # For other terrains, just record the data
                        results[terrain_name] = {
                            'passed': True,  # No specific validation criteria
                            'max_slip': max_slip,
                            'slip_ratios': slip_ratios
                        }
            
            # Clear history for next terrain test
            self.framework.debug_data_history.clear()
        
        # Overall test passes if individual terrain tests behave as expected
        overall_passed = all(result['passed'] for result in results.values())
        
        return {
            'passed': overall_passed,
            'details': {
                'terrain_results': results,
                'total_terrains_tested': len(terrain_conditions)
            }
        }
    
    def test_noise_resilience(self) -> Dict[str, Any]:
        """Test slip detection resilience to sensor noise"""
        
        noise_levels = [0.01, 0.05, 0.1, 0.2]  # Increasing noise
        results = {}
        
        for noise_level in noise_levels:
            
            linear_vel = 1.0  # m/s
            angular_vel = 0.0  # rad/s
            
            signals = []
            dt = 0.02
            
            for i in range(int(self.test_duration / dt)):
                timestamp = i * dt
                
                # IMU signal with specified noise level
                imu_msg = self.signal_gen.generate_imu_signal(
                    linear_accel=(0.0, 0.0, 0.0),
                    angular_vel=(0.0, 0.0, angular_vel),
                    noise_level=noise_level
                )
                signals.append(TestSignal(timestamp, imu_msg, 'imu'))
                
                # Joint states with noise but no actual slip
                expected_wheel_vel = linear_vel / self.wheel_radius
                wheel_positions = [expected_wheel_vel * timestamp] * 4
                wheel_velocities = [expected_wheel_vel] * 4
                
                joint_msg = self.signal_gen.generate_joint_states(
                    wheel_positions, wheel_velocities, noise_level=noise_level
                )
                signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
                
                # Command velocity
                cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
                signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
            
            # Inject signals
            self.framework.inject_signal_sequence(signals, self.test_duration)
            
            # Wait for processing
            time.sleep(1.0)
            
            # Analyze noise effects
            if self.framework.debug_data_history:
                latest_data = self.framework.debug_data_history[-1]['data']
                if len(latest_data) >= 8:
                    slip_ratios = latest_data[4:8]
                    max_slip = max(abs(slip) for slip in slip_ratios)
                    
                    # Should not detect false slip due to noise
                    false_positive = max_slip > 0.2
                    
                    results[f'noise_{noise_level}'] = {
                        'passed': not false_positive,
                        'max_slip': max_slip,
                        'false_positive': false_positive,
                        'noise_level': noise_level
                    }
            
            # Clear history for next noise level test
            self.framework.debug_data_history.clear()
        
        # Overall test passes if no false positives at any noise level
        overall_passed = all(result['passed'] for result in results.values())
        
        return {
            'passed': overall_passed,
            'details': {
                'noise_level_results': results,
                'max_acceptable_noise': max(
                    float(key.split('_')[1]) for key, result in results.items() 
                    if result['passed']
                ) if any(result['passed'] for result in results.values()) else 0.0
            }
        }
    
    def test_edge_cases(self) -> Dict[str, Any]:
        """Test edge cases for slip detection"""
        
        edge_cases = [
            ('zero_velocity', 0.0, 0.0),
            ('very_low_velocity', 0.1, 0.0),
            ('high_velocity', 3.0, 0.0),
            ('rapid_acceleration', 0.0, 0.0),  # Will add acceleration
            ('emergency_braking', 2.0, 0.0)    # Will add deceleration
        ]
        
        results = {}
        
        for case_name, initial_vel, angular_vel in edge_cases:
            
            signals = []
            dt = 0.02
            
            for i in range(int(self.test_duration / dt)):
                timestamp = i * dt
                
                # Determine velocity for this timestamp
                if case_name == 'rapid_acceleration':
                    linear_vel = min(2.0, timestamp * 2.0)  # Accelerate to 2 m/s
                    accel = 2.0 if timestamp < 1.0 else 0.0
                elif case_name == 'emergency_braking':
                    linear_vel = max(0.0, initial_vel - timestamp * 3.0)  # Brake at 3 m/s²
                    accel = -3.0 if linear_vel > 0 else 0.0
                else:
                    linear_vel = initial_vel
                    accel = 0.0
                
                # IMU signal
                imu_msg = self.signal_gen.generate_imu_signal(
                    linear_accel=(accel, 0.0, 0.0),
                    angular_vel=(0.0, 0.0, angular_vel),
                    noise_level=0.02
                )
                signals.append(TestSignal(timestamp, imu_msg, 'imu'))
                
                # Joint states
                if linear_vel > 0.01:  # Only if moving
                    expected_wheel_vel = linear_vel / self.wheel_radius
                    wheel_positions = [expected_wheel_vel * timestamp] * 4
                    wheel_velocities = [expected_wheel_vel] * 4
                else:
                    wheel_positions = [0.0] * 4
                    wheel_velocities = [0.0] * 4
                
                joint_msg = self.signal_gen.generate_joint_states(
                    wheel_positions, wheel_velocities, noise_level=0.02
                )
                signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
                
                # Command velocity
                cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
                signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
            
            # Inject signals
            self.framework.inject_signal_sequence(signals, self.test_duration)
            
            # Wait for processing
            time.sleep(1.0)
            
            # Analyze results for this edge case
            if self.framework.debug_data_history:
                latest_data = self.framework.debug_data_history[-1]['data']
                if len(latest_data) >= 8:
                    slip_ratios = latest_data[4:8]
                    max_slip = max(abs(slip) for slip in slip_ratios)
                    
                    # For edge cases, we mainly check that the system doesn't crash
                    # and produces reasonable slip estimates
                    system_stable = not any(math.isnan(slip) or math.isinf(slip) for slip in slip_ratios)
                    
                    results[case_name] = {
                        'passed': system_stable,
                        'max_slip': max_slip,
                        'slip_ratios': slip_ratios,
                        'system_stable': system_stable
                    }
            
            # Clear history for next edge case
            self.framework.debug_data_history.clear()
        
        # Overall test passes if system remains stable in all edge cases
        overall_passed = all(result['passed'] for result in results.values())
        
        return {
            'passed': overall_passed,
            'details': {
                'edge_case_results': results,
                'total_edge_cases': len(edge_cases)
            }
        }

def run_slip_detection_tests():
    """Run the complete slip detection test suite"""
    
    rclpy.init()
    
    try:
        # Create test framework
        test_framework = SignalLevelTestFramework()
        
        # Create slip detection test suite
        slip_test_suite = SlipDetectionTestSuite(test_framework)
        
        # Run all slip detection tests
        test_framework.run_test("No Slip Condition", slip_test_suite.test_no_slip_condition)
        test_framework.run_test("Single Wheel Slip", slip_test_suite.test_single_wheel_slip)
        test_framework.run_test("Turn Induced Slip", slip_test_suite.test_turn_induced_slip)
        test_framework.run_test("Terrain Based Slip", slip_test_suite.test_terrain_based_slip)
        test_framework.run_test("Noise Resilience", slip_test_suite.test_noise_resilience)
        test_framework.run_test("Edge Cases", slip_test_suite.test_edge_cases)
        
        # Generate and save test report
        report = test_framework.generate_test_report()
        test_framework.save_test_report('slip_detection_test_report.json')
        
        # Print summary
        print(f"\nSlip Detection Test Summary:")
        print(f"Total Tests: {report['summary']['total_tests']}")
        print(f"Passed: {report['summary']['passed']}")
        print(f"Failed: {report['summary']['failed']}")
        print(f"Success Rate: {report['summary']['success_rate']:.1f}%")
        print(f"Total Execution Time: {report['summary']['total_execution_time']:.2f}s")
        
        test_framework.destroy_node()
        
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    run_slip_detection_tests()