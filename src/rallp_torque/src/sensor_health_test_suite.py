#!/usr/bin/env python3
"""
Sensor Health Monitoring Test Suite

This module provides comprehensive testing for sensor timeout detection,
fallback modes, and graceful degradation when sensors fail.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
from typing import List, Dict, Any, Tuple
from signal_level_test_framework import SignalLevelTestFramework, SignalGenerator, TerrainSimulator, TestSignal

class SensorHealthTestSuite:
    """Comprehensive test suite for sensor health monitoring and fallback systems"""
    
    def __init__(self, test_framework: SignalLevelTestFramework):
        self.framework = test_framework
        self.signal_gen = SignalGenerator()
        self.terrain_sim = TerrainSimulator()
        
        # Test parameters
        self.wheel_radius = 0.125
        self.wheel_separation = 0.59
        self.max_torque = 10.0
        self.sensor_timeout = 0.5  # seconds (from config)
        self.test_duration = 6.0  # seconds per test
    
    def test_imu_timeout_detection(self) -> Dict[str, Any]:
        """Test IMU timeout detection and fallback behavior"""
        
        linear_vel = 1.0  # m/s
        angular_vel = 0.3  # rad/s
        
        signals = []
        dt = 0.02  # 50Hz
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # IMU signal - stop sending after 2 seconds to simulate timeout
            if timestamp < 2.0:
                imu_msg = self.signal_gen.generate_imu_signal(
                    linear_accel=(0.0, 0.0, 0.0),
                    angular_vel=(0.0, 0.0, angular_vel),
                    noise_level=0.01
                )
                signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states - continue sending
            expected_wheel_vel = linear_vel / self.wheel_radius
            left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            
            wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
            wheel_positions = [vel * timestamp for vel in wheel_velocities]
            
            joint_msg = self.signal_gen.generate_joint_states(
                wheel_positions, wheel_velocities, noise_level=0.01
            )
            signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
            
            # Command velocity - continue sending
            cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
            signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
        
        # Inject signals
        self.framework.inject_signal_sequence(signals, self.test_duration)
        
        # Wait for processing
        time.sleep(2.0)
        
        # Analyze fallback behavior
        passed = False
        details = {}
        
        if self.framework.diagnostics_history:
            # Check if sensor timeout was detected
            timeout_detected = False
            fallback_activated = False
            
            for diag_data in self.framework.diagnostics_history[-10:]:  # Last 10 diagnostics
                diag_msg = diag_data['data']
                for status in diag_msg.status:
                    if status.name == "torque_vectoring_system":
                        if status.level != 0:  # Not OK
                            timeout_detected = True
                        if "timeout" in status.message.lower() or "fallback" in status.message.lower():
                            fallback_activated = True
            
            # Check if system continued operating
            system_continued = len(self.framework.debug_data_history) > 0
            
            if self.framework.debug_data_history:
                latest_data = self.framework.debug_data_history[-1]['data']
                if len(latest_data) >= 4:
                    torques = latest_data[0:4]
                    system_stable = not any(math.isnan(t) or math.isinf(t) for t in torques)
                else:
                    system_stable = False
                    torques = []
            else:
                system_stable = False
                torques = []
            
            passed = timeout_detected and system_continued and system_stable
            
            details = {
                'timeout_detected': timeout_detected,
                'fallback_activated': fallback_activated,
                'system_continued': system_continued,
                'system_stable': system_stable,
                'final_torques': torques,
                'timeout_threshold': self.sensor_timeout
            }
        
        return {
            'passed': passed,
            'details': details
        }
    
    def test_joint_states_timeout_detection(self) -> Dict[str, Any]:
        """Test joint states timeout detection and fallback behavior"""
        
        linear_vel = 1.2  # m/s
        angular_vel = 0.0  # rad/s
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # IMU signal - continue sending
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(0.0, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.01
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states - stop sending after 2.5 seconds to simulate timeout
            if timestamp < 2.5:
                expected_wheel_vel = linear_vel / self.wheel_radius
                wheel_positions = [expected_wheel_vel * timestamp] * 4
                wheel_velocities = [expected_wheel_vel] * 4
                
                joint_msg = self.signal_gen.generate_joint_states(
                    wheel_positions, wheel_velocities, noise_level=0.01
                )
                signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
            
            # Command velocity - continue sending
            cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
            signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
        
        # Inject signals
        self.framework.inject_signal_sequence(signals, self.test_duration)
        
        # Wait for processing
        time.sleep(2.0)
        
        # Analyze fallback behavior for joint states timeout
        passed = False
        details = {}
        
        if self.framework.diagnostics_history:
            timeout_detected = False
            fallback_activated = False
            
            for diag_data in self.framework.diagnostics_history[-10:]:
                diag_msg = diag_data['data']
                for status in diag_msg.status:
                    if status.name == "torque_vectoring_system":
                        if status.level != 0:  # Not OK
                            timeout_detected = True
                        if "timeout" in status.message.lower() or "fallback" in status.message.lower():
                            fallback_activated = True
            
            system_continued = len(self.framework.debug_data_history) > 0
            
            if self.framework.debug_data_history:
                latest_data = self.framework.debug_data_history[-1]['data']
                if len(latest_data) >= 4:
                    torques = latest_data[0:4]
                    system_stable = not any(math.isnan(t) or math.isinf(t) for t in torques)
                else:
                    system_stable = False
                    torques = []
            else:
                system_stable = False
                torques = []
            
            passed = timeout_detected and system_continued and system_stable
            
            details = {
                'timeout_detected': timeout_detected,
                'fallback_activated': fallback_activated,
                'system_continued': system_continued,
                'system_stable': system_stable,
                'final_torques': torques,
                'sensor_type': 'joint_states'
            }
        
        return {
            'passed': passed,
            'details': details
        }
    
    def test_complete_sensor_failure(self) -> Dict[str, Any]:
        """Test behavior when all sensors fail simultaneously"""
        
        linear_vel = 1.0  # m/s
        angular_vel = 0.2  # rad/s
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # Send signals for first 1.5 seconds, then stop all sensors
            if timestamp < 1.5:
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
            
            # Command velocity - continue sending to see system response
            cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
            signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
        
        # Inject signals
        self.framework.inject_signal_sequence(signals, self.test_duration)
        
        # Wait for processing
        time.sleep(2.0)
        
        # Analyze complete failure handling
        passed = False
        details = {}
        
        if self.framework.diagnostics_history:
            complete_failure_detected = False
            emergency_mode = False
            
            for diag_data in self.framework.diagnostics_history[-10:]:
                diag_msg = diag_data['data']
                for status in diag_msg.status:
                    if status.name == "torque_vectoring_system":
                        if status.level >= 2:  # ERROR level
                            complete_failure_detected = True
                        if "fallback" in status.message.lower() or "emergency" in status.message.lower():
                            emergency_mode = True
            
            # System should detect failure and activate emergency mode
            system_responsive = len(self.framework.output_cmd_vel_history) > 0
            
            if self.framework.debug_data_history:
                latest_data = self.framework.debug_data_history[-1]['data']
                if len(latest_data) >= 4:
                    torques = latest_data[0:4]
                    # In complete failure, should use simplified control
                    torque_simplified = np.std(torques) < 1.0  # Should be more uniform
                else:
                    torques = []
                    torque_simplified = False
            else:
                torques = []
                torque_simplified = False
            
            passed = complete_failure_detected and system_responsive
            
            details = {
                'complete_failure_detected': complete_failure_detected,
                'emergency_mode': emergency_mode,
                'system_responsive': system_responsive,
                'torque_simplified': torque_simplified,
                'final_torques': torques,
                'failure_type': 'complete_sensor_failure'
            }
        
        return {
            'passed': passed,
            'details': details
        }
    
    def test_intermittent_sensor_failures(self) -> Dict[str, Any]:
        """Test handling of intermittent sensor failures"""
        
        linear_vel = 1.5  # m/s
        angular_vel = 0.4  # rad/s
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # Create intermittent failure pattern
            # IMU: Drop signals intermittently (every 10th message after t=2s)
            if timestamp < 2.0 or (timestamp >= 2.0 and i % 10 != 0):
                imu_msg = self.signal_gen.generate_imu_signal(
                    linear_accel=(0.0, 0.0, 0.0),
                    angular_vel=(0.0, 0.0, angular_vel),
                    noise_level=0.02
                )
                signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states: Drop signals intermittently (every 15th message after t=1.5s)
            if timestamp < 1.5 or (timestamp >= 1.5 and i % 15 != 0):
                expected_wheel_vel = linear_vel / self.wheel_radius
                left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
                right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
                
                wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
                wheel_positions = [vel * timestamp for vel in wheel_velocities]
                
                joint_msg = self.signal_gen.generate_joint_states(
                    wheel_positions, wheel_velocities, noise_level=0.02
                )
                signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
            
            # Command velocity - continue sending
            cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
            signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
        
        # Inject signals
        self.framework.inject_signal_sequence(signals, self.test_duration)
        
        # Wait for processing
        time.sleep(2.0)
        
        # Analyze intermittent failure handling
        passed = False
        details = {}
        
        if self.framework.debug_data_history and self.framework.diagnostics_history:
            # System should remain stable despite intermittent failures
            stability_maintained = True
            warning_count = 0
            error_count = 0
            
            # Check diagnostic status over time
            for diag_data in self.framework.diagnostics_history:
                diag_msg = diag_data['data']
                for status in diag_msg.status:
                    if status.name == "torque_vectoring_system":
                        if status.level == 1:  # WARN
                            warning_count += 1
                        elif status.level >= 2:  # ERROR
                            error_count += 1
            
            # Check torque output stability
            torque_variations = []
            for data_point in self.framework.debug_data_history:
                data = data_point['data']
                if len(data) >= 4:
                    torques = data[0:4]
                    if not any(math.isnan(t) or math.isinf(t) for t in torques):
                        torque_variations.append(np.std(torques))
                    else:
                        stability_maintained = False
                        break
            
            avg_torque_variation = np.mean(torque_variations) if torque_variations else float('inf')
            
            # System should handle intermittent failures gracefully
            reasonable_warnings = warning_count > 0 and warning_count < len(self.framework.diagnostics_history)
            no_critical_errors = error_count < len(self.framework.diagnostics_history) * 0.1
            stable_torques = avg_torque_variation < 2.0
            
            passed = stability_maintained and reasonable_warnings and no_critical_errors and stable_torques
            
            latest_data = self.framework.debug_data_history[-1]['data']
            final_torques = latest_data[0:4] if len(latest_data) >= 4 else []
            
            details = {
                'stability_maintained': stability_maintained,
                'warning_count': warning_count,
                'error_count': error_count,
                'total_diagnostics': len(self.framework.diagnostics_history),
                'avg_torque_variation': avg_torque_variation,
                'reasonable_warnings': reasonable_warnings,
                'no_critical_errors': no_critical_errors,
                'stable_torques': stable_torques,
                'final_torques': final_torques
            }
        
        return {
            'passed': passed,
            'details': details
        }
    
    def test_sensor_noise_tolerance(self) -> Dict[str, Any]:
        """Test system tolerance to high sensor noise levels"""
        
        linear_vel = 1.0  # m/s
        angular_vel = 0.3  # rad/s
        noise_levels = [0.05, 0.1, 0.2, 0.3]  # Increasing noise
        
        results = {}
        
        for noise_level in noise_levels:
            
            signals = []
            dt = 0.02
            test_duration = 3.0  # Shorter test for multiple noise levels
            
            for i in range(int(test_duration / dt)):
                timestamp = i * dt
                
                # IMU signal with high noise
                imu_msg = self.signal_gen.generate_imu_signal(
                    linear_accel=(0.0, 0.0, 0.0),
                    angular_vel=(0.0, 0.0, angular_vel),
                    noise_level=noise_level
                )
                signals.append(TestSignal(timestamp, imu_msg, 'imu'))
                
                # Joint states with high noise
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
            self.framework.inject_signal_sequence(signals, test_duration)
            time.sleep(1.0)
            
            # Analyze noise tolerance
            if self.framework.debug_data_history:
                # Check for system stability under noise
                stable_operation = True
                false_slip_detections = 0
                
                for data_point in self.framework.debug_data_history:
                    data = data_point['data']
                    if len(data) >= 8:
                        torques = data[0:4]
                        slip_ratios = data[4:8]
                        
                        # Check for NaN or infinite values
                        if any(math.isnan(t) or math.isinf(t) for t in torques):
                            stable_operation = False
                            break
                        
                        # Check for false slip detections due to noise
                        if any(abs(slip) > 0.2 for slip in slip_ratios):
                            false_slip_detections += 1
                
                false_positive_rate = (false_slip_detections / len(self.framework.debug_data_history) * 100) if self.framework.debug_data_history else 0
                
                results[f'noise_{noise_level}'] = {
                    'noise_level': noise_level,
                    'stable_operation': stable_operation,
                    'false_positive_rate': false_positive_rate,
                    'passed': stable_operation and false_positive_rate < 20.0  # Less than 20% false positives
                }
            
            # Clear history for next noise level
            self.framework.debug_data_history.clear()
            self.framework.diagnostics_history.clear()
        
        # Overall noise tolerance assessment
        overall_passed = all(result['passed'] for result in results.values())
        max_tolerated_noise = max(
            result['noise_level'] for result in results.values() if result['passed']
        ) if any(result['passed'] for result in results.values()) else 0.0
        
        return {
            'passed': overall_passed,
            'details': {
                'noise_level_results': results,
                'max_tolerated_noise': max_tolerated_noise,
                'noise_tolerance_threshold': 0.2
            }
        }
    
    def test_sensor_recovery(self) -> Dict[str, Any]:
        """Test system recovery when sensors come back online"""
        
        linear_vel = 1.3  # m/s
        angular_vel = 0.5  # rad/s
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # Phase 1 (0-1.5s): Normal operation
            # Phase 2 (1.5-3.5s): Sensor failure
            # Phase 3 (3.5s+): Sensor recovery
            
            if timestamp < 1.5 or timestamp >= 3.5:
                # Normal operation or recovery phase
                imu_msg = self.signal_gen.generate_imu_signal(
                    linear_accel=(0.0, 0.0, 0.0),
                    angular_vel=(0.0, 0.0, angular_vel),
                    noise_level=0.01
                )
                signals.append(TestSignal(timestamp, imu_msg, 'imu'))
                
                expected_wheel_vel = linear_vel / self.wheel_radius
                left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
                right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
                
                wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
                wheel_positions = [vel * timestamp for vel in wheel_velocities]
                
                joint_msg = self.signal_gen.generate_joint_states(
                    wheel_positions, wheel_velocities, noise_level=0.01
                )
                signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
            
            # Command velocity - continue throughout
            cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
            signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
        
        # Inject signals
        self.framework.inject_signal_sequence(signals, self.test_duration)
        
        # Wait for processing
        time.sleep(2.0)
        
        # Analyze sensor recovery
        passed = False
        details = {}
        
        if self.framework.diagnostics_history:
            # Check for failure detection and recovery
            failure_detected = False
            recovery_detected = False
            
            for i, diag_data in enumerate(self.framework.diagnostics_history):
                diag_msg = diag_data['data']
                timestamp = diag_data['timestamp']
                
                for status in diag_msg.status:
                    if status.name == "torque_vectoring_system":
                        # During failure period (1.5-3.5s)
                        if 1.5 <= (timestamp - self.framework.diagnostics_history[0]['timestamp']) <= 3.5:
                            if status.level != 0:  # Not OK
                                failure_detected = True
                        # During recovery period (3.5s+)
                        elif (timestamp - self.framework.diagnostics_history[0]['timestamp']) > 3.5:
                            if status.level == 0:  # OK
                                recovery_detected = True
            
            # Check system stability after recovery
            if self.framework.debug_data_history:
                post_recovery_data = [
                    data for data in self.framework.debug_data_history
                    if (data['timestamp'] - self.framework.debug_data_history[0]['timestamp']) > 4.0
                ]
                
                if post_recovery_data:
                    recovery_stable = True
                    for data_point in post_recovery_data:
                        data = data_point['data']
                        if len(data) >= 4:
                            torques = data[0:4]
                            if any(math.isnan(t) or math.isinf(t) for t in torques):
                                recovery_stable = False
                                break
                    
                    final_data = post_recovery_data[-1]['data']
                    final_torques = final_data[0:4] if len(final_data) >= 4 else []
                else:
                    recovery_stable = False
                    final_torques = []
            else:
                recovery_stable = False
                final_torques = []
            
            passed = failure_detected and recovery_detected and recovery_stable
            
            details = {
                'failure_detected': failure_detected,
                'recovery_detected': recovery_detected,
                'recovery_stable': recovery_stable,
                'final_torques': final_torques,
                'total_diagnostics': len(self.framework.diagnostics_history),
                'recovery_test_phases': ['normal', 'failure', 'recovery']
            }
        
        return {
            'passed': passed,
            'details': details
        }

def run_sensor_health_tests():
    """Run the complete sensor health monitoring test suite"""
    
    rclpy.init()
    
    try:
        # Create test framework
        test_framework = SignalLevelTestFramework()
        
        # Create sensor health test suite
        sensor_test_suite = SensorHealthTestSuite(test_framework)
        
        # Run all sensor health tests
        test_framework.run_test("IMU Timeout Detection", 
                               sensor_test_suite.test_imu_timeout_detection)
        test_framework.run_test("Joint States Timeout Detection", 
                               sensor_test_suite.test_joint_states_timeout_detection)
        test_framework.run_test("Complete Sensor Failure", 
                               sensor_test_suite.test_complete_sensor_failure)
        test_framework.run_test("Intermittent Sensor Failures", 
                               sensor_test_suite.test_intermittent_sensor_failures)
        test_framework.run_test("Sensor Noise Tolerance", 
                               sensor_test_suite.test_sensor_noise_tolerance)
        test_framework.run_test("Sensor Recovery", 
                               sensor_test_suite.test_sensor_recovery)
        
        # Generate and save test report
        report = test_framework.generate_test_report()
        test_framework.save_test_report('sensor_health_test_report.json')
        
        # Print summary
        print(f"\nSensor Health Monitoring Test Summary:")
        print(f"Total Tests: {report['summary']['total_tests']}")
        print(f"Passed: {report['summary']['passed']}")
        print(f"Failed: {report['summary']['failed']}")
        print(f"Success Rate: {report['summary']['success_rate']:.1f}%")
        print(f"Total Execution Time: {report['summary']['total_execution_time']:.2f}s")
        
        test_framework.destroy_node()
        
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    run_sensor_health_tests()