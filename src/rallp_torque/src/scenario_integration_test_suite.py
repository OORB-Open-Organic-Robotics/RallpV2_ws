#!/usr/bin/env python3
"""
Scenario-Based Integration Test Suite

This module provides comprehensive scenarios covering normal operation,
emergency conditions, and edge cases for complete system validation.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
from typing import List, Dict, Any, Tuple
from signal_level_test_framework import SignalLevelTestFramework, SignalGenerator, TerrainSimulator, TestSignal

class ScenarioIntegrationTestSuite:
    """Comprehensive test suite for scenario-based integration testing"""
    
    def __init__(self, test_framework: SignalLevelTestFramework):
        self.framework = test_framework
        self.signal_gen = SignalGenerator()
        self.terrain_sim = TerrainSimulator()
        
        # Test parameters
        self.wheel_radius = 0.125
        self.wheel_separation = 0.59
        self.max_torque = 10.0
        self.power_limit = 100.0
        self.test_duration = 8.0  # Longer duration for complex scenarios
    
    def test_highway_driving_scenario(self) -> Dict[str, Any]:
        """Test highway driving scenario with lane changes and varying speeds"""
        
        signals = []
        dt = 0.02  # 50Hz
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # Highway driving profile
            if timestamp < 2.0:
                # Phase 1: Acceleration to highway speed
                linear_vel = 0.5 + timestamp * 1.0  # Accelerate to 2.5 m/s
                angular_vel = 0.0
                accel = 1.0
            elif timestamp < 4.0:
                # Phase 2: Cruising with minor steering adjustments
                linear_vel = 2.5
                angular_vel = 0.1 * math.sin((timestamp - 2.0) * 2.0)  # Small oscillations
                accel = 0.0
            elif timestamp < 6.0:
                # Phase 3: Lane change maneuver
                linear_vel = 2.5
                angular_vel = 0.6 * math.sin((timestamp - 4.0) * math.pi / 2.0)  # Smooth lane change
                accel = 0.0
            else:
                # Phase 4: Return to cruise and slight deceleration
                linear_vel = max(2.0, 2.5 - (timestamp - 6.0) * 0.25)
                angular_vel = 0.0
                accel = -0.25
            
            # IMU signal
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(accel, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.02
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states - normal highway conditions
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
        self.framework.inject_signal_sequence(signals, self.test_duration)
        
        # Wait for processing
        time.sleep(2.0)
        
        # Analyze highway driving performance
        passed = False
        details = {}
        
        if self.framework.debug_data_history:
            # Check system stability throughout scenario
            stable_operation = True
            power_efficiency = []
            torque_smoothness = []
            
            for data_point in self.framework.debug_data_history:
                data = data_point['data']
                if len(data) >= 13:
                    torques = data[0:4]
                    power = data[12]
                    
                    # Check for stability
                    if any(math.isnan(t) or math.isinf(t) for t in torques):
                        stable_operation = False
                        break
                    
                    # Track power efficiency
                    power_efficiency.append(power / self.power_limit)
                    
                    # Track torque smoothness
                    torque_smoothness.append(np.std(torques))
            
            avg_power_efficiency = np.mean(power_efficiency) if power_efficiency else 0
            avg_torque_smoothness = np.mean(torque_smoothness) if torque_smoothness else 0
            
            # Highway driving should be smooth and efficient
            efficient_operation = avg_power_efficiency < 0.7  # Less than 70% power usage
            smooth_torques = avg_torque_smoothness < 1.5
            no_false_slips = True
            
            # Check for unnecessary slip detections
            if len(self.framework.debug_data_history[-10:]) > 0:
                for data_point in self.framework.debug_data_history[-10:]:
                    data = data_point['data']
                    if len(data) >= 8:
                        slip_ratios = data[4:8]
                        if any(abs(slip) > 0.15 for slip in slip_ratios):
                            no_false_slips = False
                            break
            
            passed = stable_operation and efficient_operation and smooth_torques and no_false_slips
            
            final_data = self.framework.debug_data_history[-1]['data']
            details = {
                'stable_operation': stable_operation,
                'avg_power_efficiency': avg_power_efficiency,
                'avg_torque_smoothness': avg_torque_smoothness,
                'efficient_operation': efficient_operation,
                'smooth_torques': smooth_torques,
                'no_false_slips': no_false_slips,
                'final_torques': final_data[0:4] if len(final_data) >= 4 else [],
                'scenario_phases': ['acceleration', 'cruise', 'lane_change', 'deceleration']
            }
        
        return {
            'passed': passed,
            'details': details
        }
    
    def test_urban_stop_and_go_scenario(self) -> Dict[str, Any]:
        """Test urban driving with frequent stops, starts, and turns"""
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # Urban stop-and-go profile
            cycle_time = timestamp % 4.0  # 4-second cycles
            
            if cycle_time < 1.0:
                # Acceleration phase
                linear_vel = cycle_time * 1.5  # Accelerate to 1.5 m/s
                angular_vel = 0.0
                accel = 1.5
            elif cycle_time < 2.0:
                # Turning while moving
                linear_vel = 1.5
                angular_vel = 0.8 * math.sin((cycle_time - 1.0) * math.pi)  # Turn and straighten
                accel = 0.0
            elif cycle_time < 3.0:
                # Deceleration phase
                linear_vel = max(0.0, 1.5 - (cycle_time - 2.0) * 1.5)
                angular_vel = 0.0
                accel = -1.5
            else:
                # Stop phase
                linear_vel = 0.0
                angular_vel = 0.0
                accel = 0.0
            
            # IMU signal
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(accel, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.03  # Higher noise in urban environment
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states
            if linear_vel > 0.01:
                left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
                right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
                wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
            else:
                wheel_velocities = [0.0, 0.0, 0.0, 0.0]
            
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
        time.sleep(2.0)
        
        # Analyze urban driving performance
        passed = False
        details = {}
        
        if self.framework.debug_data_history:
            stable_transitions = True
            stop_handling = True
            turn_assistance = True
            
            # Analyze different phases
            acceleration_phases = []
            turning_phases = []
            stop_phases = []
            
            for data_point in self.framework.debug_data_history:
                timestamp = data_point['timestamp']
                data = data_point['data']
                cycle_time = timestamp % 4.0
                
                if len(data) >= 8:
                    torques = data[0:4]
                    slip_ratios = data[4:8]
                    
                    # Check for stability
                    if any(math.isnan(t) or math.isinf(t) for t in torques):
                        stable_transitions = False
                        break
                    
                    # Categorize by phase
                    if cycle_time < 1.0:  # Acceleration
                        acceleration_phases.append(torques)
                    elif 1.0 <= cycle_time < 2.0:  # Turning
                        turning_phases.append(torques)
                    elif cycle_time >= 3.0:  # Stop
                        stop_phases.append(torques)
                        # During stops, torques should be minimal
                        if any(abs(t) > 1.0 for t in torques):
                            stop_handling = False
            
            # Check torque vectoring during turns
            if turning_phases:
                turn_torque_variations = [np.std(torques) for torques in turning_phases]
                avg_turn_variation = np.mean(turn_torque_variations)
                turn_assistance = avg_turn_variation > 0.5  # Should show differential torques
            
            passed = stable_transitions and stop_handling and turn_assistance
            
            final_data = self.framework.debug_data_history[-1]['data']
            details = {
                'stable_transitions': stable_transitions,
                'stop_handling': stop_handling,
                'turn_assistance': turn_assistance,
                'acceleration_phases_count': len(acceleration_phases),
                'turning_phases_count': len(turning_phases),
                'stop_phases_count': len(stop_phases),
                'final_torques': final_data[0:4] if len(final_data) >= 4 else [],
                'scenario_type': 'urban_stop_and_go'
            }
        
        return {
            'passed': passed,
            'details': details
        }
    
    def test_off_road_terrain_scenario(self) -> Dict[str, Any]:
        """Test off-road driving with varying terrain and slip conditions"""
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # Off-road terrain profile
            linear_vel = 1.0  # Constant moderate speed
            angular_vel = 0.2 * math.sin(timestamp * 0.5)  # Gentle steering for terrain following
            
            # Determine terrain type based on time
            if timestamp < 2.0:
                terrain = self.terrain_sim.normal_terrain()
            elif timestamp < 4.0:
                terrain = self.terrain_sim.slippery_terrain()
            elif timestamp < 6.0:
                terrain = self.terrain_sim.high_resistance_terrain()
            else:
                terrain = self.terrain_sim.mixed_terrain()
            
            # IMU signal with terrain-based noise
            noise_level = terrain.get('noise_level', 0.02)
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(0.0, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=noise_level
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states with terrain effects
            left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            
            base_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
            
            # Apply terrain effects
            if 'wheel_conditions' in terrain:  # Mixed terrain
                wheel_velocities = []
                for j, wheel_config in enumerate(terrain['wheel_conditions']):
                    slip_coeff = wheel_config['slip_coefficient']
                    resistance = wheel_config['resistance_factor']
                    actual_vel = base_velocities[j] * (1.0 + slip_coeff) / resistance
                    wheel_velocities.append(actual_vel)
            else:  # Uniform terrain
                slip_coeff = terrain['slip_coefficient']
                resistance = terrain['resistance_factor']
                wheel_velocities = [vel * (1.0 + slip_coeff) / resistance for vel in base_velocities]
            
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
        time.sleep(2.0)
        
        # Analyze off-road performance
        passed = False
        details = {}
        
        if self.framework.debug_data_history:
            slip_detection_phases = {
                'normal': 0,
                'slippery': 0,
                'high_resistance': 0,
                'mixed': 0
            }
            
            terrain_adaptation = True
            traction_control_active = False
            
            for data_point in self.framework.debug_data_history:
                timestamp = data_point['timestamp']
                data = data_point['data']
                
                if len(data) >= 8:
                    torques = data[0:4]
                    slip_ratios = data[4:8]
                    
                    # Determine terrain phase
                    if timestamp < 2.0:
                        phase = 'normal'
                    elif timestamp < 4.0:
                        phase = 'slippery'
                    elif timestamp < 6.0:
                        phase = 'high_resistance'
                    else:
                        phase = 'mixed'
                    
                    # Check slip detection in each phase
                    max_slip = max(abs(slip) for slip in slip_ratios)
                    if max_slip > 0.15:
                        slip_detection_phases[phase] += 1
                        traction_control_active = True
                    
                    # Check for reasonable torque adaptation
                    torque_std = np.std(torques)
                    if phase in ['slippery', 'mixed'] and torque_std < 0.1:
                        terrain_adaptation = False  # Should show adaptation
            
            # System should detect slip in challenging terrains
            challenging_terrain_slip = (slip_detection_phases['slippery'] > 0 or 
                                      slip_detection_phases['mixed'] > 0)
            
            # Should maintain stability throughout
            stable_operation = all(
                not any(math.isnan(data['data'][i]) or math.isinf(data['data'][i]) 
                       for i in range(min(4, len(data['data']))))
                for data in self.framework.debug_data_history
                if len(data['data']) >= 4
            )
            
            passed = challenging_terrain_slip and terrain_adaptation and stable_operation
            
            final_data = self.framework.debug_data_history[-1]['data']
            details = {
                'slip_detection_phases': slip_detection_phases,
                'challenging_terrain_slip': challenging_terrain_slip,
                'terrain_adaptation': terrain_adaptation,
                'traction_control_active': traction_control_active,
                'stable_operation': stable_operation,
                'final_torques': final_data[0:4] if len(final_data) >= 4 else [],
                'terrain_phases': ['normal', 'slippery', 'high_resistance', 'mixed']
            }
        
        return {
            'passed': passed,
            'details': details
        }
    
    def test_emergency_maneuver_scenario(self) -> Dict[str, Any]:
        """Test emergency maneuvers like obstacle avoidance and emergency braking"""
        
        signals = []
        dt = 0.02
        
        for i in range(int(self.test_duration / dt)):
            timestamp = i * dt
            
            # Emergency maneuver profile
            if timestamp < 2.0:
                # Phase 1: Normal driving
                linear_vel = 2.0
                angular_vel = 0.0
                accel = 0.0
            elif timestamp < 3.0:
                # Phase 2: Sudden obstacle avoidance (sharp turn)
                linear_vel = 2.0
                angular_vel = 1.2 * math.sin((timestamp - 2.0) * math.pi)  # Sharp S-turn
                accel = 0.0
            elif timestamp < 4.5:
                # Phase 3: Emergency braking
                decel_time = timestamp - 3.0
                linear_vel = max(0.0, 2.0 - decel_time * 4.0)  # Hard braking
                angular_vel = 0.0
                accel = -4.0 if linear_vel > 0 else 0.0
            else:
                # Phase 4: Recovery and gentle acceleration
                linear_vel = min(1.0, (timestamp - 4.5) * 0.5)
                angular_vel = 0.0
                accel = 0.5 if timestamp < 6.5 else 0.0
            
            # IMU signal with high dynamics
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(accel, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.03
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states with potential slip during emergency maneuvers
            if linear_vel > 0.01:
                left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
                right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
                wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
                
                # Add slip during aggressive maneuvers
                if abs(angular_vel) > 0.8 or abs(accel) > 3.0:
                    # Outer wheels might slip during sharp turns
                    if angular_vel > 0.8:  # Left turn
                        wheel_velocities[1] *= 1.3  # FR wheel
                        wheel_velocities[3] *= 1.2  # RR wheel
                    elif angular_vel < -0.8:  # Right turn
                        wheel_velocities[0] *= 1.3  # FL wheel
                        wheel_velocities[2] *= 1.2  # RL wheel
                    
                    # Front wheels might slip during hard braking
                    if accel < -3.0:
                        wheel_velocities[0] *= 0.8  # FL wheel
                        wheel_velocities[1] *= 0.8  # FR wheel
            else:
                wheel_velocities = [0.0, 0.0, 0.0, 0.0]
            
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
        time.sleep(2.0)
        
        # Analyze emergency maneuver performance
        passed = False
        details = {}
        
        if self.framework.debug_data_history:
            emergency_stability = True
            slip_management = False
            power_management = True
            recovery_success = True
            
            emergency_phases = {
                'avoidance': [],  # 2-3s
                'braking': [],    # 3-4.5s
                'recovery': []    # 4.5s+
            }
            
            for data_point in self.framework.debug_data_history:
                timestamp = data_point['timestamp']
                data = data_point['data']
                
                if len(data) >= 13:
                    torques = data[0:4]
                    slip_ratios = data[4:8]
                    power = data[12]
                    
                    # Check for system stability
                    if any(math.isnan(t) or math.isinf(t) for t in torques):
                        emergency_stability = False
                        break
                    
                    # Categorize by emergency phase
                    if 2.0 <= timestamp < 3.0:
                        emergency_phases['avoidance'].append({
                            'torques': torques,
                            'slip_ratios': slip_ratios,
                            'power': power
                        })
                    elif 3.0 <= timestamp < 4.5:
                        emergency_phases['braking'].append({
                            'torques': torques,
                            'slip_ratios': slip_ratios,
                            'power': power
                        })
                    elif timestamp >= 4.5:
                        emergency_phases['recovery'].append({
                            'torques': torques,
                            'slip_ratios': slip_ratios,
                            'power': power
                        })
                    
                    # Check slip management during emergency
                    max_slip = max(abs(slip) for slip in slip_ratios)
                    if max_slip > 0.2 and timestamp >= 2.0:  # During emergency phases
                        slip_management = True
                    
                    # Check power management
                    if power > self.power_limit * 1.1:  # Allow some tolerance during emergency
                        power_management = False
            
            # Check recovery phase stability
            if emergency_phases['recovery']:
                recovery_torques = [phase['torques'] for phase in emergency_phases['recovery']]
                if recovery_torques:
                    recovery_stability = all(
                        not any(math.isnan(t) or math.isinf(t) for t in torques)
                        for torques in recovery_torques
                    )
                    recovery_success = recovery_stability
            
            passed = emergency_stability and slip_management and power_management and recovery_success
            
            final_data = self.framework.debug_data_history[-1]['data']
            details = {
                'emergency_stability': emergency_stability,
                'slip_management': slip_management,
                'power_management': power_management,
                'recovery_success': recovery_success,
                'avoidance_events': len(emergency_phases['avoidance']),
                'braking_events': len(emergency_phases['braking']),
                'recovery_events': len(emergency_phases['recovery']),
                'final_torques': final_data[0:4] if len(final_data) >= 4 else [],
                'emergency_phases': ['normal', 'avoidance', 'braking', 'recovery']
            }
        
        return {
            'passed': passed,
            'details': details
        }
    
    def test_long_duration_stability(self) -> Dict[str, Any]:
        """Test system stability over extended operation"""
        
        extended_duration = 12.0  # 12 seconds for stability test
        signals = []
        dt = 0.02
        
        for i in range(int(extended_duration / dt)):
            timestamp = i * dt
            
            # Varied but realistic driving pattern
            cycle = timestamp % 6.0  # 6-second cycles
            
            if cycle < 2.0:
                linear_vel = 1.0 + 0.5 * math.sin(timestamp * 0.3)
                angular_vel = 0.3 * math.cos(timestamp * 0.4)
            elif cycle < 4.0:
                linear_vel = 1.5
                angular_vel = 0.5 * math.sin((cycle - 2.0) * math.pi)
            else:
                linear_vel = 2.0
                angular_vel = 0.1 * math.sin(timestamp * 0.2)
            
            # IMU signal
            imu_msg = self.signal_gen.generate_imu_signal(
                linear_accel=(0.0, 0.0, 0.0),
                angular_vel=(0.0, 0.0, angular_vel),
                noise_level=0.02
            )
            signals.append(TestSignal(timestamp, imu_msg, 'imu'))
            
            # Joint states with occasional minor slip
            left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
            
            wheel_velocities = [left_wheel_vel, right_wheel_vel, left_wheel_vel, right_wheel_vel]
            
            # Add occasional slip
            if int(timestamp) % 8 == 0 and (timestamp % 1.0) < 0.5:  # Every 8 seconds for 0.5s
                wheel_velocities[1] *= 1.1  # Minor FR wheel slip
            
            wheel_positions = [vel * timestamp for vel in wheel_velocities]
            
            joint_msg = self.signal_gen.generate_joint_states(
                wheel_positions, wheel_velocities, noise_level=0.02
            )
            signals.append(TestSignal(timestamp, joint_msg, 'joint_states'))
            
            # Command velocity
            cmd_vel_msg = self.signal_gen.generate_cmd_vel(linear_vel, angular_vel)
            signals.append(TestSignal(timestamp, cmd_vel_msg, 'cmd_vel'))
        
        # Inject signals
        self.framework.inject_signal_sequence(signals, extended_duration)
        
        # Wait for processing
        time.sleep(3.0)
        
        # Analyze long-term stability
        passed = False
        details = {}
        
        if self.framework.debug_data_history:
            # Check for consistent operation
            consistent_operation = True
            power_consistency = []
            torque_consistency = []
            error_count = 0
            
            for data_point in self.framework.debug_data_history:
                data = data_point['data']
                if len(data) >= 13:
                    torques = data[0:4]
                    power = data[12]
                    
                    # Check for errors
                    if any(math.isnan(t) or math.isinf(t) for t in torques):
                        error_count += 1
                        if error_count > len(self.framework.debug_data_history) * 0.01:  # >1% error rate
                            consistent_operation = False
                            break
                    
                    power_consistency.append(power)
                    torque_consistency.append(np.mean(np.abs(torques)))
            
            # Check for drift or degradation
            if power_consistency and len(power_consistency) > 100:
                first_quarter = power_consistency[:len(power_consistency)//4]
                last_quarter = power_consistency[-len(power_consistency)//4:]
                
                power_drift = abs(np.mean(last_quarter) - np.mean(first_quarter))
                stable_power = power_drift < self.power_limit * 0.1  # Less than 10% drift
            else:
                stable_power = True
                power_drift = 0.0
            
            if torque_consistency and len(torque_consistency) > 100:
                torque_stability = np.std(torque_consistency) < 1.0
            else:
                torque_stability = True
            
            # Check diagnostic health over time
            diagnostic_health = True
            if self.framework.diagnostics_history:
                error_diagnostics = sum(
                    1 for diag in self.framework.diagnostics_history
                    for status in diag['data'].status
                    if status.name == "torque_vectoring_system" and status.level >= 2
                )
                diagnostic_health = error_diagnostics < len(self.framework.diagnostics_history) * 0.05  # <5% errors
            
            passed = consistent_operation and stable_power and torque_stability and diagnostic_health
            
            final_data = self.framework.debug_data_history[-1]['data']
            details = {
                'consistent_operation': consistent_operation,
                'stable_power': stable_power,
                'torque_stability': torque_stability,
                'diagnostic_health': diagnostic_health,
                'error_count': error_count,
                'total_data_points': len(self.framework.debug_data_history),
                'power_drift': power_drift,
                'test_duration': extended_duration,
                'final_torques': final_data[0:4] if len(final_data) >= 4 else []
            }
        
        return {
            'passed': passed,
            'details': details
        }

def run_scenario_integration_tests():
    """Run the complete scenario-based integration test suite"""
    
    rclpy.init()
    
    try:
        # Create test framework
        test_framework = SignalLevelTestFramework()
        
        # Create scenario integration test suite
        scenario_test_suite = ScenarioIntegrationTestSuite(test_framework)
        
        # Run all scenario integration tests
        test_framework.run_test("Highway Driving Scenario", 
                               scenario_test_suite.test_highway_driving_scenario)
        test_framework.run_test("Urban Stop and Go Scenario", 
                               scenario_test_suite.test_urban_stop_and_go_scenario)
        test_framework.run_test("Off-Road Terrain Scenario", 
                               scenario_test_suite.test_off_road_terrain_scenario)
        test_framework.run_test("Emergency Maneuver Scenario", 
                               scenario_test_suite.test_emergency_maneuver_scenario)
        test_framework.run_test("Long Duration Stability", 
                               scenario_test_suite.test_long_duration_stability)
        
        # Generate and save test report
        report = test_framework.generate_test_report()
        test_framework.save_test_report('scenario_integration_test_report.json')
        
        # Print summary
        print(f"\nScenario Integration Test Summary:")
        print(f"Total Tests: {report['summary']['total_tests']}")
        print(f"Passed: {report['summary']['passed']}")
        print(f"Failed: {report['summary']['failed']}")
        print(f"Success Rate: {report['summary']['success_rate']:.1f}%")
        print(f"Total Execution Time: {report['summary']['total_execution_time']:.2f}s")
        
        test_framework.destroy_node()
        
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    run_scenario_integration_tests()