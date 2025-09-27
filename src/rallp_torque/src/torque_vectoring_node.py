#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64, Float64MultiArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import numpy as np
import time
from collections import deque
import math

class TorqueVectoringNode(Node):
    def __init__(self):
        super().__init__('torque_vectoring_node')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.125)
        self.declare_parameter('wheel_separation', 0.59)
        self.declare_parameter('wheelbase', 0.49)
        self.declare_parameter('max_torque', 10.0)
        self.declare_parameter('slip_threshold', 0.2)
        self.declare_parameter('slip_detection_window', 0.1)
        self.declare_parameter('enable_torque_vectoring', True)
        self.declare_parameter('vectoring_gain', 0.5)
        self.declare_parameter('power_limit', 100.0)
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_torque = self.get_parameter('max_torque').value
        self.slip_threshold = self.get_parameter('slip_threshold').value
        self.slip_detection_window = self.get_parameter('slip_detection_window').value
        self.enable_torque_vectoring = self.get_parameter('enable_torque_vectoring').value
        self.vectoring_gain = self.get_parameter('vectoring_gain').value
        self.power_limit = self.get_parameter('power_limit').value
        
        # Initialize wheel data
        self.wheel_names = ['fl', 'fr', 'rl', 'rr']
        self.wheel_positions = np.zeros(4)
        self.wheel_velocities = np.zeros(4)
        self.wheel_torques = np.zeros(4)
        self.slip_ratios = np.zeros(4)
        self.prev_wheel_positions = np.zeros(4)
        
        # IMU data
        self.imu_angular_velocity_z = 0.0
        self.imu_linear_acceleration_x = 0.0
        
        # Command data
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        
        # Timestamps
        self.last_imu_time = time.time()
        self.last_joint_time = time.time()
        
        # State tracking
        self.sensors_healthy = True
        self.total_power_consumption = 0.0
        self.terrain_type = "unknown"
        
        # Slip detection history
        self.slip_history = [deque(maxlen=10) for _ in range(4)]
        
        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel_raw', self.cmd_vel_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        
        # Create publishers for torque vectoring output
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create publishers for individual wheel velocity monitoring (debug only)
        self.wheel_torque_pubs = [
            self.create_publisher(Float64, '/debug/fl_joint/target_vel', 10),
            self.create_publisher(Float64, '/debug/fr_joint/target_vel', 10),
            self.create_publisher(Float64, '/debug/rl_joint/target_vel', 10),
            self.create_publisher(Float64, '/debug/rr_joint/target_vel', 10)
        ]
        
        # Create debug and diagnostics publishers
        self.debug_pub = self.create_publisher(Float64MultiArray, '/vectoring/debug', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        
        # Create timer for main control loop
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz
        
        self.get_logger().info("Torque Vectoring Node initialized (Python)")
        self.get_logger().info(f"Torque vectoring {'ENABLED' if self.enable_torque_vectoring else 'DISABLED'}")

    def cmd_vel_callback(self, msg):
        self.target_linear_vel = msg.linear.x
        self.target_angular_vel = msg.angular.z

    def imu_callback(self, msg):
        self.imu_angular_velocity_z = msg.angular_velocity.z
        self.imu_linear_acceleration_x = msg.linear_acceleration.x
        self.last_imu_time = time.time()

    def joint_states_callback(self, msg):
        # Store previous positions for slip detection
        self.prev_wheel_positions = self.wheel_positions.copy()
        
        # Update wheel states
        for i, name in enumerate(msg.name):
            for j, wheel in enumerate(self.wheel_names):
                if f'{wheel}_joint' in name:
                    if i < len(msg.position):
                        self.wheel_positions[j] = msg.position[i]
                    if i < len(msg.velocity):
                        self.wheel_velocities[j] = msg.velocity[i]
                    break
        
        self.last_joint_time = time.time()

    def control_loop(self):
        # Check sensor health
        self.check_sensor_health()
        
        if not self.sensors_healthy:
            self.apply_fallback_control()
            return
        
        # Detect wheel slippage
        self.detect_wheel_slippage()
        
        # Calculate base wheel velocities from differential drive kinematics
        left_vel = self.target_linear_vel - (self.target_angular_vel * self.wheel_separation / 2.0)
        right_vel = self.target_linear_vel + (self.target_angular_vel * self.wheel_separation / 2.0)
        
        # Base torque commands
        base_torques = np.array([
            left_vel * self.max_torque / 2.0,   # fl
            right_vel * self.max_torque / 2.0,  # fr
            left_vel * self.max_torque / 2.0,   # rl
            right_vel * self.max_torque / 2.0   # rr
        ])
        
        if self.enable_torque_vectoring:
            # Apply torque vectoring algorithm
            self.wheel_torques = self.apply_torque_vectoring(base_torques)
        else:
            self.wheel_torques = base_torques
        
        # Apply power limit
        self.apply_power_limit()
        
        # Publish torque commands
        self.publish_torque_commands()
        
        # Publish debug information
        self.publish_debug_info()
        
        # Publish diagnostics
        self.publish_diagnostics()
        
        # Update power consumption tracking
        self.update_power_consumption()

    def check_sensor_health(self):
        current_time = time.time()
        
        # Check IMU timeout
        imu_healthy = (current_time - self.last_imu_time) < 0.5
        
        # Check joint states timeout
        joints_healthy = (current_time - self.last_joint_time) < 0.5
        
        self.sensors_healthy = imu_healthy and joints_healthy
        
        if not self.sensors_healthy:
            self.get_logger().warn(
                f"Sensor health check failed - IMU: {'OK' if imu_healthy else 'TIMEOUT'}, "
                f"Joints: {'OK' if joints_healthy else 'TIMEOUT'}",
                throttle_duration_sec=1.0)

    def detect_wheel_slippage(self):
        # Basic slip detection using encoder delta vs IMU integration
        dt = 0.02  # 50 Hz control rate
        
        # Calculate expected wheel velocities from IMU
        expected_linear_vel = self.imu_linear_acceleration_x * dt  # Simplified integration
        expected_left_wheel_vel = (expected_linear_vel - self.imu_angular_velocity_z * self.wheel_separation / 2.0) / self.wheel_radius
        expected_right_wheel_vel = (expected_linear_vel + self.imu_angular_velocity_z * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Calculate slip ratios
        expected_vels = [expected_left_wheel_vel, expected_right_wheel_vel, 
                        expected_left_wheel_vel, expected_right_wheel_vel]
        
        for i in range(4):
            slip_ratio = self.calculate_slip_ratio(self.wheel_velocities[i], expected_vels[i])
            self.slip_ratios[i] = slip_ratio
            self.slip_history[i].append(slip_ratio)

    def calculate_slip_ratio(self, actual_wheel_vel, expected_wheel_vel):
        if abs(expected_wheel_vel) < 0.01:
            return 0.0  # No slip when not moving
        return (actual_wheel_vel - expected_wheel_vel) / abs(expected_wheel_vel)

    def apply_torque_vectoring(self, base_torques):
        torques = base_torques.copy()
        
        # 1. Slip compensation: reduce torque on slipping wheels
        for i in range(4):
            if abs(self.slip_ratios[i]) > self.slip_threshold:
                slip_factor = 1.0 - min(abs(self.slip_ratios[i]), 1.0) * self.vectoring_gain
                torques[i] *= slip_factor
                
                self.get_logger().info(
                    f"Wheel {self.wheel_names[i]} slipping ({self.slip_ratios[i]:.2f}), "
                    f"reducing torque by {(1.0 - slip_factor) * 100:.1f}%",
                    throttle_duration_sec=0.5)
        
        # 2. Cornering assistance: increase outer wheel torque during turns
        if abs(self.target_angular_vel) > 0.1:
            turn_direction = 1.0 if self.target_angular_vel > 0 else -1.0  # +1 for left turn, -1 for right turn
            cornering_boost = abs(self.target_angular_vel) * self.vectoring_gain
            
            if turn_direction > 0:  # Left turn - boost right wheels
                torques[1] *= (1.0 + cornering_boost)  # fr
                torques[3] *= (1.0 + cornering_boost)  # rr
            else:  # Right turn - boost left wheels
                torques[0] *= (1.0 + cornering_boost)  # fl
                torques[2] *= (1.0 + cornering_boost)  # rl
        
        # 3. Traction control: redistribute torque from slipping to gripping wheels
        torques = self.redistribute_torque_from_slipping(torques)
        
        return torques

    def redistribute_torque_from_slipping(self, torques):
        total_available_torque = 0.0
        total_redistributed_torque = 0.0
        is_slipping = np.zeros(4, dtype=bool)
        
        # Identify slipping wheels and calculate available torque
        for i in range(4):
            is_slipping[i] = abs(self.slip_ratios[i]) > self.slip_threshold
            if is_slipping[i]:
                total_redistributed_torque += torques[i] * 0.3  # Take 30% of torque from slipping wheels
                torques[i] *= 0.7  # Reduce slipping wheel torque
            else:
                total_available_torque += self.max_torque - abs(torques[i])
        
        # Redistribute torque to non-slipping wheels
        if total_available_torque > 0 and total_redistributed_torque > 0:
            for i in range(4):
                if not is_slipping[i]:
                    boost_ratio = (self.max_torque - abs(torques[i])) / total_available_torque
                    boost = total_redistributed_torque * boost_ratio
                    
                    if torques[i] >= 0:
                        torques[i] += boost
                    else:
                        torques[i] -= boost
        
        return torques

    def apply_power_limit(self):
        # Calculate total power consumption (simplified)
        total_power = np.sum(np.abs(self.wheel_torques * self.wheel_velocities))
        
        # Scale down torques if power limit exceeded
        if total_power > self.power_limit:
            scale_factor = self.power_limit / total_power
            self.wheel_torques *= scale_factor
            self.get_logger().warn(
                f"Power limit exceeded ({total_power:.1f}W), scaling torques by {scale_factor:.2f}",
                throttle_duration_sec=1.0)
        
        self.total_power_consumption = min(total_power, self.power_limit)

    def apply_fallback_control(self):
        # Simple fallback: equal torque distribution
        base_torque = self.target_linear_vel * self.max_torque * 0.5
        self.wheel_torques = np.full(4, base_torque)
        
        self.publish_torque_commands()
        
        self.get_logger().warn(
            "Running in fallback mode - sensors unhealthy",
            throttle_duration_sec=2.0)

    def publish_torque_commands(self):
        # Convert wheel torques back to linear and angular velocities
        # This simulates torque vectoring by adjusting the cmd_vel output
        
        # Calculate average left and right wheel torques
        left_torque = (self.wheel_torques[0] + self.wheel_torques[2]) / 2.0  # FL + RL
        right_torque = (self.wheel_torques[1] + self.wheel_torques[3]) / 2.0  # FR + RR
        
        # Convert torques to velocities (simplified model)
        left_velocity = left_torque / self.max_torque * 2.0   # Scale to reasonable velocity
        right_velocity = right_torque / self.max_torque * 2.0
        
        # Calculate linear and angular velocities from wheel velocities
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = (left_velocity + right_velocity) / 2.0
        cmd_vel_msg.angular.z = (right_velocity - left_velocity) / self.wheel_separation
        
        # Publish the modified cmd_vel
        self.cmd_vel_pub.publish(cmd_vel_msg)
        
        # Publish debug information about individual wheel targets
        for i, torque in enumerate(self.wheel_torques):
            msg = Float64()
            msg.data = float(torque / self.max_torque)  # Normalized torque for debugging
            self.wheel_torque_pubs[i].publish(msg)

    def publish_debug_info(self):
        debug_msg = Float64MultiArray()
        
        # Pack debug data: [wheel_torques(4), slip_ratios(4), wheel_velocities(4), power, angular_vel, linear_vel, sensors_healthy]
        debug_data = np.concatenate([
            self.wheel_torques,
            self.slip_ratios,
            self.wheel_velocities,
            [self.total_power_consumption, self.imu_angular_velocity_z, 
             self.target_linear_vel, 1.0 if self.sensors_healthy else 0.0]
        ])
        
        debug_msg.data = debug_data.tolist()
        self.debug_pub.publish(debug_msg)

    def publish_diagnostics(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Main system status
        main_status = DiagnosticStatus()
        main_status.name = "torque_vectoring_system"
        main_status.level = DiagnosticStatus.OK if self.sensors_healthy else DiagnosticStatus.WARN
        main_status.message = "System operational" if self.sensors_healthy else "Sensor timeout detected"
        
        # Add key-value pairs
        main_status.values.append(KeyValue(key="torque_vectoring_enabled", value=str(self.enable_torque_vectoring)))
        main_status.values.append(KeyValue(key="power_consumption_watts", value=f"{self.total_power_consumption:.1f}"))
        main_status.values.append(KeyValue(key="terrain_type", value=self.terrain_type))
        
        # Add slip ratios
        for i, wheel in enumerate(self.wheel_names):
            main_status.values.append(KeyValue(key=f"slip_ratio_{wheel}", value=f"{self.slip_ratios[i]:.3f}"))
        
        diag_array.status.append(main_status)
        self.diagnostics_pub.publish(diag_array)

    def update_power_consumption(self):
        # Simple terrain classification based on power consumption and slip
        avg_slip = np.mean(np.abs(self.slip_ratios))
        
        if avg_slip > 0.3 and self.total_power_consumption > self.power_limit * 0.8:
            self.terrain_type = "slippery_high_resistance"
        elif avg_slip > 0.2:
            self.terrain_type = "slippery"
        elif self.total_power_consumption > self.power_limit * 0.6:
            self.terrain_type = "high_resistance"
        else:
            self.terrain_type = "normal"
        
        # Log power and torque distribution
        self.get_logger().info(
            f"Power: {self.total_power_consumption:.1f}W, Terrain: {self.terrain_type}, "
            f"Avg Slip: {avg_slip:.3f}, Torques: [{self.wheel_torques[0]:.2f}, "
            f"{self.wheel_torques[1]:.2f}, {self.wheel_torques[2]:.2f}, {self.wheel_torques[3]:.2f}]",
            throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = TorqueVectoringNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
