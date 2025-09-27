#!/usr/bin/env python3
"""
Power Consumption and Limiting Test Suite

This module provides comprehensive testing for power calculation, limiting,
and safety systems with various load conditions and power constraints.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
from typing import List, Dict, Any, Tuple
from signal_level_test_framework import SignalLevelTestFramework, SignalGenerator, TerrainSimulator, TestSignal

class PowerManagementTestSuite:
    """Comprehensive test suite for power management and limiting"""
    
    def __init__(self, test_framework: SignalLevelTestFramework):
        self.framework = test_framework
        self.signal_gen = SignalGenerator()
        self.terrain_sim = TerrainSimulator()
        
        # Test parameters
        self.wheel_radius = 0.125
        self.wheel_separation = 0.59
        self.max_torque = 10.0
        self.power_limit = 100.0  # Watts (from config)
        self.test_duration = 4.0  # seconds per test
    
    def test_normal_power_consumption(self) -> Dict[str, Any]:
        """Test power consumption calculation under normal operating conditions"""
        
        linear_vel = 1.0  # m/s
        angular_vel = 0.0  # rad/s
        
        signals = []
        dt = 0.02  # 50Hz
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # IMU signal
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(0.0, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.01
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states - normal operation
            expected_wheel_vel = linear_vel / self.wheel_radius
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
        
        # Validate power consumption is reasonable and within limits
        power_within_limits = self.framework.validate_power_consumption(self.power_limit)
        
        details = {}
        if self.framework.debug_data_history:
            latest_data = self.framework.debug_data_history[-1]['data']
            if len(latest_data) >= 13:
                power_consumption = latest_data[12]
                torques = latest_data[0:4]
                velocities = latest_data[8:12] if len(latest_data) >= 12 else [0]*4
                
                # Calculate expected power (simplified: P = T * ω)
                expected_power = sum(abs(torques[i] * velocities[i]) for i in range(4))
                
                details = {
                    'measured_power': power_consumption,
                    'expected_power': expected_power,
                    'power_limit': self.power_limit,
                    'within_limits': power_consumption <= self.power_limit,
                    'power_efficiency': (expected_power / self.power_limit * 100) if self.power_limit > 0 else 0,
                    'torques': torques,
                    'velocities': velocities
                }
        
        return {
            'passed': power_within_limits,
            'details': details
        }
    
    def test_high_load_power_limiting(self) -> Dict[str, Any]:
        """Test power limiting under high load conditions"""
        
        linear_vel = 2.5  # m/s (high speed)
        angular_vel = 0.8  # rad/s (sharp turn)
        
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
            
            # Joint states - high speed differential motion
            left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            
            wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
            wheel_positions = [vel * timestamp for vel in wheel_velocities]
            
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
        
        # Validate power limiting is active
        power_limited = self.framework.validate_power_consumption(self.power_limit)
        
        details = {}
        if self.framework.debug_data_history:
            latest_data = self.framework.debug_data_history[-1]['data']
            if len(latest_data) >= 13:
                power_consumption = latest_data[12]
                torques = latest_data[0:4]
                velocities = latest_data[8:12] if len(latest_data) >= 12 else [0]*4
                
                # Check if power limiting was necessary
                unlimited_power = sum(abs(self.max_torque * velocities[i]) for i in range(4))
                power_reduction = max(0, unlimited_power - self.power_limit)
                limiting_active = power_consumption < unlimited_power and power_consumption <= self.power_limit
                
                details = {
                    'measured_power': power_consumption,
                    'unlimited_power': unlimited_power,
                    'power_limit': self.power_limit,
                    'power_reduction': power_reduction,
                    'limiting_active': limiting_active,
                    'power_utilization': (power_consumption / self.power_limit * 100) if self.power_limit > 0 else 0,
                    'torques': torques,
                    'velocities': velocities
                }
        
        return {
            'passed': power_limited,
            'details': details
        }
    
    def test_power_spike_handling(self) -> Dict[str, Any]:
        """Test handling of sudden power spikes"""
        
        # Scenario: Sudden acceleration + turn (power spike scenario)
        base_linear_vel = 0.5  # m/s
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # Create power spike scenario
            if timestamp < 1.0:
                # Phase 1: Normal operation
                linear_vel = base_linear_vel
                angular_vel = 0.0
                accel = 0.0
            elif timestamp < 2.0:
                # Phase 2: Sudden acceleration + turn (power spike)
                linear_vel = base_linear_vel + (timestamp - 1.0) * 3.0  # Rapid acceleration
                angular_vel = 1.2  # Sharp turn
                accel = 3.0
            else:
                # Phase 3: Maintain high speed
                linear_vel = 3.5
                angular_vel = 0.5
                accel = 0.0
            
            # IMU signal
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(accel, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.02
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states
            left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            
            wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
            wheel_positions = [vel * timestamp for vel in wheel_velocities]
            
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
        
        # Analyze power spike handling
        passed = True
        power_peaks = []
        power_violations = 0
        
        details = {}
        if self.framework.debug_data_history:
            # Analyze power consumption over time
            for data_point in self.framework.debug_data_history:
                data = data_point['data']
                if len(data) >= 13:
                    power = data[12]
                    power_peaks.append(power)
                    if power > self.power_limit * 1.05:  # 5% tolerance
                        power_violations += 1
            
            max_power = max(power_peaks) if power_peaks else 0
            avg_power = np.mean(power_peaks) if power_peaks else 0
            
            # System should handle spikes without significant violations
            passed = power_violations < len(power_peaks) * 0.1  # Less than 10% violations
            
            latest_data = self.framework.debug_data_history[-1]['data']
            if len(latest_data) >= 4:
                details = {
                    'max_power_observed': max_power,
                    'avg_power': avg_power,
                    'power_limit': self.power_limit,
                    'power_violations': power_violations,
                    'total_samples': len(power_peaks),
                    'violation_rate': (power_violations / len(power_peaks) * 100) if power_peaks else 0,
                    'spike_handled': passed,
                    'final_torques': latest_data[0:4]
                }
        
        return {
            'passed': passed,
            'details': details
        }
    
    def test_varying_power_limits(self) -> Dict[str, Any]:
        """Test system behavior with different power limit configurations"""
        
        power_limits = [50.0, 75.0, 100.0, 150.0]  # Different power limits to test
        linear_vel = 2.0  # m/s (constant high load)
        angular_vel = 0.0  # rad/s
        
        results = {}
        
        for power_limit in power_limits:
            # For this test, we simulate different power limits
            # In reality, we'd need to reconfigure the node parameter
            
            signals = []
            dt = 0.02
            test_duration = 2.0  # Shorter test for multiple limits
            
            for i in range(int(test_duration / dt)):
                timestamp = i * dt
                
                # IMU signal
                imu_msg = self.signal_gen.generate_imu_signal(
                    linear_accel=(0.0, 0.0, 0.0),
                    angular_vel=(0.0, 0.0, angular_vel),
                    noise_level=0.01
                )
                signals.append(TestSignal(timestamp, imu_msg, 'imu'))
                
                # Joint states
                expected_wheel_vel = linear_vel / self.wheel_radius
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
            self.framework.inject_signal_sequence(signals, test_duration)
            time.sleep(1.0)
            
            # Analyze results for this power limit
            if self.framework.debug_data_history:
                latest_data = self.framework.debug_data_history[-1]['data']
                if len(latest_data) >= 13:
                    measured_power = latest_data[12]
                    torques = latest_data[0:4]
                    
                    # Simulate expected behavior for different limits
                    expected_limiting = measured_power >= self.power_limit * 0.9
                    
                    results[f'limit_{power_limit}W'] = {
                        'power_limit': power_limit,
                        'measured_power': measured_power,
                        'torques': torques,
                        'limiting_expected': expected_limiting,
                        'within_limit': measured_power <= self.power_limit
                    }
            
            # Clear history for next limit test
            self.framework.debug_data_history.clear()
        
        # Overall assessment
        overall_passed = all(result['within_limit'] for result in results.values())
        
        return {
            'passed': overall_passed,
            'details': {
                'power_limit_results': results,
                'note': 'This test simulates behavior - actual implementation would require parameter reconfiguration'
            }
        }
    
    def test_power_efficiency_optimization(self) -> Dict[str, Any]:
        """Test power efficiency optimization during different driving patterns"""
        
        driving_patterns = [
            ('constant_speed', 1.5, 0.0),
            ('city_driving', None, None),  # Variable pattern
            ('highway_cruise', 2.5, 0.0),
            ('parking_maneuvers', 0.5, 0.8)
        ]
        
        results = {}
        
        for pattern_name, base_linear, base_angular in driving_patterns:
            
            signals = []
            dt = 0.02
            test_duration = 3.0
            
            for i in range(int(test_duration / dt)):
                timestamp = i * dt
                
                # Define driving pattern
                if pattern_name == 'city_driving':
                    # Variable speed city driving
                    linear_vel = 1.0 + 0.5 * math.sin(timestamp * 0.5)
                    angular_vel = 0.3 * math.sin(timestamp * 0.8)
                else:
                    linear_vel = base_linear
                    angular_vel = base_angular
                
                # IMU signal
                imu_msg = self.signal_gen.generate_imu_signal(
                    linear_accel=(0.0, 0.0, 0.0),
                    angular_vel=(0.0, 0.0, angular_vel),
                    noise_level=0.01
                )
                signals.append(TestSignal(timestamp, imu_msg, 'imu'))
                
                # Joint states
                left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
                right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
                
                wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
                wheel_positions = [vel * timestamp for vel in wheel_velocities]
                
                joint_msg = self.signal_gen.generate_joint_states(
                    wheel_positions, wheel_velocities, noise_level=0.01
                )
                signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
                
                # Command velocity
                cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
                signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
            
            # Inject signals
            self.framework.inject_signal_sequence(signals, test_duration)
            time.sleep(1.0)
            
            # Analyze power efficiency for this pattern
            if self.framework.debug_data_history:
                power_samples = []
                for data_point in self.framework.debug_data_history:
                    data = data_point['data']
                    if len(data) >= 13:
                        power_samples.append(data[12])
                
                if power_samples:
                    avg_power = np.mean(power_samples)
                    max_power = max(power_samples)
                    power_efficiency = (avg_power / self.power_limit * 100) if self.power_limit > 0 else 0
                    
                    results[pattern_name] = {
                        'avg_power': avg_power,
                        'max_power': max_power,
                        'power_efficiency': power_efficiency,
                        'power_stability': np.std(power_samples) if len(power_samples) > 1 else 0
                    }
            
            # Clear history for next pattern
            self.framework.debug_data_history.clear()
        
        # Overall efficiency assessment
        if results:
            efficiency_scores = [result['power_efficiency'] for result in results.values()]
            avg_efficiency = np.mean(efficiency_scores)
            passed = avg_efficiency < 90.0  # Should not constantly use >90% of power limit
        else:
            passed = False
            avg_efficiency = 0
        
        return {
            'passed': passed,
            'details': {
                'driving_pattern_results': results,
                'average_efficiency': avg_efficiency,
                'efficiency_threshold': 90.0
            }
        }
    
    def test_emergency_power_management(self) -> Dict[str, Any]:
        """Test emergency power management scenarios"""
        
        # Scenario: System overload requiring emergency power reduction
        linear_vel = 3.0  # m/s (very high speed)
        angular_vel = 1.5  # rad/s (very sharp turn)
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # Create emergency scenario with slip and high power demand
            if timestamp > 1.0:
                # Add slip to increase power demand
                slip_factor = 0.6  # 60% slip
            else:
                slip_factor = 0.0
            
            # IMU signal
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(0.0, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.03
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states with emergency conditions
            left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            
            wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
            
            # Add slip-induced power demand
            if slip_factor > 0:
                wheel_velocities[0] *= (1.0 + slip_factor)  # FL
                wheel_velocities[1] *= (1.0 + slip_factor)  # FR
            
            wheel_positions = [vel * timestamp for vel in wheel_velocities]
            
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
        
        # Analyze emergency power management
        passed = False
        details = {}
        
        if self.framework.debug_data_history:
            power_samples = []
            max_violations = 0
            
            for data_point in self.framework.debug_data_history:
                data = data_point['data']
                if len(data) >= 13:
                    power = data[12]
                    power_samples.append(power)
                    if power > self.power_limit * 1.1:  # 10% tolerance for emergency
                        max_violations += 1
            
            if power_samples:
                max_power = max(power_samples)
                avg_power = np.mean(power_samples)
                
                # Emergency management should prevent catastrophic power spikes
                emergency_handled = max_violations < len(power_samples) * 0.05  # Less than 5% violations
                power_controlled = max_power < self.power_limit * 1.2  # Within 20% of limit
                
                passed = emergency_handled and power_controlled
                
                latest_data = self.framework.debug_data_history[-1]['data']
                if len(latest_data) >= 8:
                    details = {
                        'max_power': max_power,
                        'avg_power': avg_power,
                        'power_limit': self.power_limit,
                        'max_violations': max_violations,
                        'total_samples': len(power_samples),
                        'emergency_handled': emergency_handled,
                        'power_controlled': power_controlled,
                        'final_torques': latest_data[0:4],
                        'final_slip_ratios': latest_data[4:8] if len(latest_data) >= 8 else [0]*4
                    }
        
        return {
            'passed': passed,
            'details': details
        }

def run_power_management_tests():
    """Run the complete power management test suite"""
    
    rclpy.init()
    
    try:
        # Create test framework
        test_framework = SignalLevelTestFramework()
        
        # Create power management test suite
        power_test_suite = PowerManagementTestSuite(test_framework)
        
        # Run all power management tests
        test_framework.run_test("Normal Power Consumption", 
                               power_test_suite.test_normal_power_consumption)
        test_framework.run_test("High Load Power Limiting", 
                               power_test_suite.test_high_load_power_limiting)
        test_framework.run_test("Power Spike Handling", 
                               power_test_suite.test_power_spike_handling)
        test_framework.run_test("Varying Power Limits", 
                               power_test_suite.test_varying_power_limits)
        test_framework.run_test("Power Efficiency Optimization", 
                               power_test_suite.test_power_efficiency_optimization)
        test_framework.run_test("Emergency Power Management", 
                               power_test_suite.test_emergency_power_management)
        
        # Generate and save test report
        report = test_framework.generate_test_report()
        test_framework.save_test_report('power_management_test_report.json')
        
        # Print summary
        print(f"\nPower Management Test Summary:")
        print(f"Total Tests: {report['summary']['total_tests']}")
        print(f"Passed: {report['summary']['passed']}")
        print(f"Failed: {report['summary']['failed']}")
        print(f"Success Rate: {report['summary']['success_rate']:.1f}%")
        print(f"Total Execution Time: {report['summary']['total_execution_time']:.2f}s")
        
        test_framework.destroy_node()
        
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    run_power_management_tests()