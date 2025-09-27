#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from diagnostic_msgs.msg import DiagnosticArray
import time
import math

class TorqueVectoringTester(Node):
    def __init__(self):
        super().__init__('torque_vectoring_tester')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribers for monitoring
        self.debug_sub = self.create_subscription(
            Float64MultiArray, '/vectoring/debug', self.debug_callback, 10)
        
        self.diagnostics_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10)
        
        # Test data storage
        self.last_debug_data = None
        self.last_diagnostics = None
        
        # Test sequence timer
        self.test_timer = self.create_timer(2.0, self.run_test_sequence)
        self.test_phase = 0
        self.start_time = time.time()
        
        self.get_logger().info("Torque Vectoring Tester started")
        self.get_logger().info("Running test sequence to validate torque vectoring system...")

    def debug_callback(self, msg):
        self.last_debug_data = msg.data
        if len(msg.data) >= 16:
            torques = msg.data[0:4]
            slip_ratios = msg.data[4:8]
            power = msg.data[12]
            self.get_logger().info(
                f"Torques: [{torques[0]:.2f}, {torques[1]:.2f}, {torques[2]:.2f}, {torques[3]:.2f}] "
                f"Power: {power:.1f}W",
                throttle_duration_sec=1.0)

    def diagnostics_callback(self, msg):
        self.last_diagnostics = msg
        for status in msg.status:
            if status.name == "torque_vectoring_system":
                level_str = ["OK", "WARN", "ERROR", "STALE"][status.level]
                self.get_logger().info(
                    f"System Status: {level_str} - {status.message}",
                    throttle_duration_sec=2.0)

    def run_test_sequence(self):
        twist = Twist()
        
        if self.test_phase == 0:
            # Test 1: Forward motion
            self.get_logger().info("Test Phase 1: Forward motion")
            twist.linear.x = 1.0
            twist.angular.z = 0.0
            
        elif self.test_phase == 1:
            # Test 2: Left turn (should activate torque vectoring)
            self.get_logger().info("Test Phase 2: Left turn (torque vectoring test)")
            twist.linear.x = 0.5
            twist.angular.z = 1.0
            
        elif self.test_phase == 2:
            # Test 3: Right turn
            self.get_logger().info("Test Phase 3: Right turn")
            twist.linear.x = 0.5
            twist.angular.z = -1.0
            
        elif self.test_phase == 3:
            # Test 4: Spin in place
            self.get_logger().info("Test Phase 4: Spin in place")
            twist.linear.x = 0.0
            twist.angular.z = 2.0
            
        elif self.test_phase == 4:
            # Test 5: Stop
            self.get_logger().info("Test Phase 5: Stop")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            
        elif self.test_phase == 5:
            # Test 6: Reverse
            self.get_logger().info("Test Phase 6: Reverse")
            twist.linear.x = -0.5
            twist.angular.z = 0.0
            
        else:
            # Test complete
            self.get_logger().info("Test sequence complete!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.test_timer.cancel()
            
            # Print summary
            self.print_test_summary()
            
        self.cmd_vel_pub.publish(twist)
        self.test_phase += 1

    def print_test_summary(self):
        self.get_logger().info("=" * 50)
        self.get_logger().info("TORQUE VECTORING TEST SUMMARY")
        self.get_logger().info("=" * 50)
        
        if self.last_debug_data and len(self.last_debug_data) >= 16:
            torques = self.last_debug_data[0:4]
            slip_ratios = self.last_debug_data[4:8]
            power = self.last_debug_data[12]
            sensors_healthy = self.last_debug_data[15] > 0.5
            
            self.get_logger().info(f"Final torques: FL={torques[0]:.2f}, FR={torques[1]:.2f}, RL={torques[2]:.2f}, RR={torques[3]:.2f}")
            self.get_logger().info(f"Final slip ratios: FL={slip_ratios[0]:.3f}, FR={slip_ratios[1]:.3f}, RL={slip_ratios[2]:.3f}, RR={slip_ratios[3]:.3f}")
            self.get_logger().info(f"Power consumption: {power:.1f}W")
            self.get_logger().info(f"Sensors healthy: {sensors_healthy}")
            
            # Check if torque vectoring is working
            torque_variation = max(torques) - min(torques)
            if torque_variation > 0.1:
                self.get_logger().info("✓ PASS: Torque vectoring is active (torque variation detected)")
            else:
                self.get_logger().warn("⚠ WARNING: Limited torque variation detected")
                
            # Check slip detection
            max_slip = max(abs(s) for s in slip_ratios)
            if max_slip > 0.01:
                self.get_logger().info("✓ PASS: Slip detection is working")
            else:
                self.get_logger().info("ℹ INFO: No significant slip detected (expected on flat ground)")
                
        if self.last_diagnostics:
            self.get_logger().info("✓ PASS: Diagnostics system is operational")
        else:
            self.get_logger().warn("⚠ WARNING: No diagnostics received")
            
        self.get_logger().info("=" * 50)
        self.get_logger().info("Test completed. Check the results above.")
        self.get_logger().info("Use 'ros2 launch rallp torque_vectoring_launch.py' to start the visualizer for real-time monitoring.")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TorqueVectoringTester()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
