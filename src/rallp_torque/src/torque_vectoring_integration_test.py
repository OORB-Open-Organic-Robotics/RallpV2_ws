#!/usr/bin/env python3
"""
Complete Integration Test for Torque Vectoring System
Tests the full system integration including URDF, Gazebo, and ROS2 components
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticArray
import time
import threading

class TorqueVectoringIntegrationTest(Node):
    def __init__(self):
        super().__init__('torque_vectoring_integration_test')
        
        # Test status tracking
        self.joint_states_received = False
        self.imu_data_received = False
        self.torque_commands_working = False
        self.diagnostics_received = False
        
        # Data storage
        self.latest_joint_states = None
        self.latest_imu = None
        self.latest_diagnostics = None
        
        # Create subscribers to monitor system
        self.joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)
        
        # Monitor individual wheel torque commands (debug topics)
        self.wheel_effort_subs = []
        self.wheel_efforts = [0.0, 0.0, 0.0, 0.0]
        
        for i, joint in enumerate(['fl_joint', 'fr_joint', 'rl_joint', 'rr_joint']):
            sub = self.create_subscription(
                Float64, f'/debug/{joint}/target_vel', 
                lambda msg, idx=i: self.wheel_effort_callback(msg, idx), 10)
            self.wheel_effort_subs.append(sub)
        
        # Monitor the final cmd_vel output from torque vectoring
        self.cmd_vel_output_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_output_callback, 10)
            
        self.latest_cmd_vel_output = None
        
        # Create publisher for test commands (raw input to torque vectoring)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        
        self.get_logger().info("Torque Vectoring Integration Test initialized")
    
    def joint_states_callback(self, msg):
        self.joint_states_received = True
        self.latest_joint_states = msg
    
    def imu_callback(self, msg):
        self.imu_data_received = True
        self.latest_imu = msg
    
    def diagnostics_callback(self, msg):
        self.diagnostics_received = True
        self.latest_diagnostics = msg
    
    def cmd_vel_output_callback(self, msg):
        self.latest_cmd_vel_output = msg
        
    def wheel_effort_callback(self, msg, wheel_idx):
        self.wheel_efforts[wheel_idx] = msg.data
        self.torque_commands_working = True
    
    def run_integration_tests(self):
        """Run comprehensive integration tests"""
        
        self.get_logger().info("🧪 STARTING TORQUE VECTORING INTEGRATION TESTS")
        self.get_logger().info("=" * 60)
        
        # Test 1: Sensor Data Reception
        self.get_logger().info("Test 1: Checking sensor data reception...")
        time.sleep(2.0)  # Wait for initial data
        
        result1 = self.check_sensor_data()
        
        # Test 2: Command Response
        self.get_logger().info("Test 2: Testing command response...")
        result2 = self.test_command_response()
        
        # Test 3: Torque Vectoring Logic
        self.get_logger().info("Test 3: Testing torque vectoring logic...")
        result3 = self.test_torque_vectoring_logic()
        
        # Test 4: System Health
        self.get_logger().info("Test 4: Checking system health...")
        result4 = self.check_system_health()
        
        # Final Results
        self.print_test_results([result1, result2, result3, result4])
        
        return all([result1, result2, result3, result4])
    
    def check_sensor_data(self):
        """Test 1: Verify all sensor data is being received"""
        
        # Check joint states
        joint_test = self.joint_states_received and self.latest_joint_states is not None
        if joint_test:
            expected_joints = ['fl_joint', 'fr_joint', 'rl_joint', 'rr_joint']
            joints_present = all(joint in self.latest_joint_states.name for joint in expected_joints)
            joint_test = joint_test and joints_present
        
        # Check IMU data
        imu_test = self.imu_data_received and self.latest_imu is not None
        
        self.get_logger().info(f"  Joint States: {'✅ PASS' if joint_test else '❌ FAIL'}")
        self.get_logger().info(f"  IMU Data: {'✅ PASS' if imu_test else '❌ FAIL'}")
        
        if joint_test and self.latest_joint_states:
            self.get_logger().info(f"  Detected joints: {self.latest_joint_states.name}")
        
        return joint_test and imu_test
    
    def test_command_response(self):
        """Test 2: Send commands and verify torque vectoring responds"""
        
        # Send straight movement command
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.0
        
        self.get_logger().info("  Sending straight movement command...")
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.0)
        
        # Check if torque commands are being published
        straight_test = self.torque_commands_working and any(effort != 0.0 for effort in self.wheel_efforts)
        
        # Send turning command
        twist.linear.x = 0.5
        twist.angular.z = 1.0
        
        self.get_logger().info("  Sending turning command...")
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.0)
        
        # Check if turning affects torque distribution
        left_torque = (self.wheel_efforts[0] + self.wheel_efforts[2]) / 2
        right_torque = (self.wheel_efforts[1] + self.wheel_efforts[3]) / 2
        turning_test = abs(left_torque - right_torque) > 0.01  # Should be different for turning
        
        self.get_logger().info(f"  Command Response: {'✅ PASS' if straight_test else '❌ FAIL'}")
        self.get_logger().info(f"  Turning Logic: {'✅ PASS' if turning_test else '❌ FAIL'}")
        
        return straight_test and turning_test
    
    def test_torque_vectoring_logic(self):
        """Test 3: Verify torque vectoring algorithms are working"""
        
        # Test differential torque for turning
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 2.0  # Sharp turn
        
        self.get_logger().info("  Testing sharp turn differential...")
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.5)
        
        # Check torque distribution
        fl, fr, rl, rr = self.wheel_efforts
        left_total = fl + rl
        right_total = fr + rr
        
        differential_test = abs(left_total - right_total) > 0.5
        
        # Test power limits
        max_effort = max(abs(effort) for effort in self.wheel_efforts)
        power_limit_test = max_effort <= 15.0  # Should respect configured limits
        
        self.get_logger().info(f"  Differential Logic: {'✅ PASS' if differential_test else '❌ FAIL'}")
        self.get_logger().info(f"  Power Limits: {'✅ PASS' if power_limit_test else '❌ FAIL'}")
        self.get_logger().info(f"  Current efforts: FL={fl:.2f}, FR={fr:.2f}, RL={rl:.2f}, RR={rr:.2f}")
        
        return differential_test and power_limit_test
    
    def check_system_health(self):
        """Test 4: Check system health and diagnostics"""
        
        # Wait for diagnostics
        time.sleep(1.0)
        
        diagnostics_test = self.diagnostics_received and self.latest_diagnostics is not None
        
        system_healthy = True
        if diagnostics_test and self.latest_diagnostics:
            for status in self.latest_diagnostics.status:
                if status.level > 1:  # ERROR or higher
                    system_healthy = False
                    self.get_logger().warn(f"  System issue: {status.name} - {status.message}")
        
        # Check sensor timeouts
        current_time = time.time()
        sensor_timeout_test = True  # Would need timestamp comparison in real implementation
        
        self.get_logger().info(f"  Diagnostics: {'✅ PASS' if diagnostics_test else '❌ FAIL'}")
        self.get_logger().info(f"  System Health: {'✅ PASS' if system_healthy else '❌ FAIL'}")
        
        return diagnostics_test and system_healthy
    
    def print_test_results(self, results):
        """Print final test results summary"""
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🎯 INTEGRATION TEST RESULTS")
        self.get_logger().info("=" * 60)
        
        test_names = [
            "Sensor Data Reception",
            "Command Response", 
            "Torque Vectoring Logic",
            "System Health"
        ]
        
        passed = sum(results)
        total = len(results)
        
        for i, (name, result) in enumerate(zip(test_names, results)):
            self.get_logger().info(f"Test {i+1}: {name:<25} {'✅ PASS' if result else '❌ FAIL'}")
        
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"Overall Result: {passed}/{total} tests passed")
        
        if passed == total:
            self.get_logger().info("🎉 ALL TESTS PASSED - SYSTEM READY FOR DEPLOYMENT!")
        else:
            self.get_logger().warn("⚠️  Some tests failed - check configuration and try again")
        
        self.get_logger().info("=" * 60)


def main():
    rclpy.init()
    
    test_node = TorqueVectoringIntegrationTest()
    
    def run_tests():
        time.sleep(3)  # Wait for system to fully initialize
        success = test_node.run_integration_tests()
        
        # Keep node alive for a bit more monitoring
        time.sleep(5)
        rclpy.shutdown()
    
    # Run tests in separate thread
    test_thread = threading.Thread(target=run_tests)
    test_thread.start()
    
    # Spin the node
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    
    test_thread.join()
    test_node.destroy_node()


if __name__ == '__main__':
    main()
