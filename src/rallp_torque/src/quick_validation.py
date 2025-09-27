#!/usr/bin/env python3
"""
Quick Test Validation Script for Torque Vectoring System
Performs rapid validation of key torque vectoring functionality
"""

import sys
import os
import time
import subprocess
from pathlib import Path

# Add src directory to Python path
current_dir = Path(__file__).parent
src_dir = current_dir
sys.path.insert(0, str(src_dir))

def print_status(message, status="INFO"):
    """Print formatted status message"""
    colors = {
        "INFO": "\033[0;34m",     # Blue
        "SUCCESS": "\033[0;32m",  # Green  
        "WARNING": "\033[1;33m",  # Yellow
        "ERROR": "\033[0;31m",    # Red
        "RESET": "\033[0m"        # Reset
    }
    
    symbols = {
        "INFO": "ℹ️ ",
        "SUCCESS": "✅",
        "WARNING": "⚠️ ",
        "ERROR": "❌"
    }
    
    color = colors.get(status, colors["INFO"])
    symbol = symbols.get(status, "")
    reset = colors["RESET"]
    
    print(f"{color}{symbol} {message}{reset}")

def check_dependencies():
    """Check if required dependencies are available"""
    print_status("Checking dependencies...", "INFO")
    
    # Check ROS2 environment
    if not os.environ.get('ROS_VERSION'):
        print_status("ROS2 environment not detected", "WARNING")
        return False
        
    # Check Python modules
    required_modules = ['rclpy', 'geometry_msgs', 'sensor_msgs', 'numpy']
    missing_modules = []
    
    for module in required_modules:
        try:
            __import__(module)
        except ImportError:
            missing_modules.append(module)
    
    if missing_modules:
        print_status(f"Missing Python modules: {', '.join(missing_modules)}", "WARNING")
        return False
        
    # Check torque vectoring node
    torque_node_path = src_dir / "torque_vectoring_node.py"
    if not torque_node_path.exists():
        print_status("Torque vectoring node not found", "WARNING")
        return False
        
    print_status("All dependencies available", "SUCCESS")
    return True

def quick_slip_detection_test():
    """Quick slip detection validation"""
    print_status("Running quick slip detection test...", "INFO")
    
    try:
        from signal_level_test_framework import SignalLevelTestFramework
        
        framework = SignalLevelTestFramework()
        
        # Generate test signal with slip condition
        test_signal = framework.signal_generator.generate_slip_scenario(
            duration=2.0,
            slip_wheels=[0, 1],  # Front wheels slipping
            slip_intensity=0.8
        )
        
        # Inject signal and validate
        framework.inject_test_signal(test_signal)
        time.sleep(0.5)  # Allow processing
        
        # Check slip detection
        slip_detected = framework.validate_slip_detection(
            expected_slip_wheels=[0, 1],
            tolerance=0.1
        )
        
        if slip_detected:
            print_status("Slip detection test PASSED", "SUCCESS")
            return True
        else:
            print_status("Slip detection test FAILED", "ERROR")
            return False
            
    except Exception as e:
        print_status(f"Slip detection test ERROR: {str(e)}", "ERROR")
        return False

def quick_torque_redistribution_test():
    """Quick torque redistribution validation"""
    print_status("Running quick torque redistribution test...", "INFO")
    
    try:
        from signal_level_test_framework import SignalLevelTestFramework
        
        framework = SignalLevelTestFramework()
        
        # Generate test signal with left turn
        test_signal = framework.signal_generator.generate_turn_scenario(
            duration=2.0,
            turn_direction="left",
            turn_intensity=0.6
        )
        
        # Inject signal and validate
        framework.inject_test_signal(test_signal)
        time.sleep(0.5)  # Allow processing
        
        # Check torque redistribution (left wheels should get more torque)
        torque_valid = framework.validate_torque_distribution(
            expected_pattern="left_boost",
            tolerance=0.1
        )
        
        if torque_valid:
            print_status("Torque redistribution test PASSED", "SUCCESS")
            return True
        else:
            print_status("Torque redistribution test FAILED", "ERROR")
            return False
            
    except Exception as e:
        print_status(f"Torque redistribution test ERROR: {str(e)}", "ERROR")
        return False

def quick_power_management_test():
    """Quick power management validation"""
    print_status("Running quick power management test...", "INFO")
    
    try:
        from signal_level_test_framework import SignalLevelTestFramework
        
        framework = SignalLevelTestFramework()
        
        # Generate test signal with high power demand
        test_signal = framework.signal_generator.generate_high_power_scenario(
            duration=2.0,
            power_demand=120.0  # Above 100W limit
        )
        
        # Inject signal and validate
        framework.inject_test_signal(test_signal)
        time.sleep(0.5)  # Allow processing
        
        # Check power limiting
        power_limited = framework.validate_power_management(
            max_power=100.0,
            tolerance=5.0
        )
        
        if power_limited:
            print_status("Power management test PASSED", "SUCCESS")
            return True
        else:
            print_status("Power management test FAILED", "ERROR")
            return False
            
    except Exception as e:
        print_status(f"Power management test ERROR: {str(e)}", "ERROR")
        return False

def quick_sensor_health_test():
    """Quick sensor health validation"""
    print_status("Running quick sensor health test...", "INFO")
    
    try:
        from signal_level_test_framework import SignalLevelTestFramework
        
        framework = SignalLevelTestFramework()
        
        # Simulate sensor timeout
        test_signal = framework.signal_generator.generate_sensor_timeout_scenario(
            duration=2.0,
            timeout_sensors=['imu'],
            timeout_duration=1.0
        )
        
        # Inject signal and validate
        framework.inject_test_signal(test_signal)
        time.sleep(1.5)  # Allow timeout to trigger
        
        # Check sensor health monitoring
        health_detected = framework.validate_sensor_health(
            expected_timeouts=['imu'],
            tolerance=0.1
        )
        
        if health_detected:
            print_status("Sensor health test PASSED", "SUCCESS")
            return True
        else:
            print_status("Sensor health test FAILED", "ERROR")
            return False
            
    except Exception as e:
        print_status(f"Sensor health test ERROR: {str(e)}", "ERROR")
        return False

def quick_integration_test():
    """Quick integration test"""
    print_status("Running quick integration test...", "INFO")
    
    try:
        from signal_level_test_framework import SignalLevelTestFramework
        
        framework = SignalLevelTestFramework()
        
        # Generate combined scenario
        test_signal = framework.signal_generator.generate_combined_scenario(
            duration=3.0,
            include_slip=True,
            include_turns=True,
            include_power_variation=True
        )
        
        # Inject signal and validate
        framework.inject_test_signal(test_signal)
        time.sleep(2.0)  # Allow processing
        
        # Check overall system response
        system_stable = framework.validate_system_stability(
            tolerance=0.1
        )
        
        if system_stable:
            print_status("Integration test PASSED", "SUCCESS")
            return True
        else:
            print_status("Integration test FAILED", "ERROR")
            return False
            
    except Exception as e:
        print_status(f"Integration test ERROR: {str(e)}", "ERROR")
        return False

def run_comprehensive_test():
    """Run comprehensive automated test suite"""
    print_status("Running comprehensive automated test suite...", "INFO")
    
    try:
        # Use the automated test runner
        script_path = src_dir / "automated_test_runner.py"
        if not script_path.exists():
            print_status("Automated test runner not found", "ERROR")
            return False
            
        # Run with quick configuration
        result = subprocess.run([
            sys.executable, str(script_path),
            "--suites", "slip_detection", "torque_redistribution",
            "--report-dir", str(current_dir.parent / "test_reports")
        ], capture_output=True, text=True, timeout=120)
        
        if result.returncode == 0:
            print_status("Comprehensive test suite PASSED", "SUCCESS")
            return True
        else:
            print_status(f"Comprehensive test suite FAILED (exit code: {result.returncode})", "ERROR")
            if result.stderr:
                print_status(f"Error output: {result.stderr[:200]}...", "ERROR")
            return False
            
    except subprocess.TimeoutExpired:
        print_status("Comprehensive test suite TIMEOUT", "ERROR")
        return False
    except Exception as e:
        print_status(f"Comprehensive test suite ERROR: {str(e)}", "ERROR")
        return False

def main():
    """Main validation function"""
    print_status("🚗 RALLP_V2 Torque Vectoring Quick Validation", "INFO")
    print("=" * 50)
    
    start_time = time.time()
    
    # Check dependencies
    if not check_dependencies():
        print_status("Dependency check failed - some tests may not work properly", "WARNING")
    
    # Define test sequence
    quick_tests = [
        ("Slip Detection", quick_slip_detection_test),
        ("Torque Redistribution", quick_torque_redistribution_test), 
        ("Power Management", quick_power_management_test),
        ("Sensor Health", quick_sensor_health_test),
        ("Integration", quick_integration_test)
    ]
    
    # Run quick tests
    print_status("Running quick validation tests...", "INFO")
    passed_tests = 0
    total_tests = len(quick_tests)
    
    for test_name, test_func in quick_tests:
        try:
            if test_func():
                passed_tests += 1
        except Exception as e:
            print_status(f"{test_name} test encountered error: {str(e)}", "ERROR")
    
    print()
    print_status(f"Quick tests completed: {passed_tests}/{total_tests} passed", 
                 "SUCCESS" if passed_tests == total_tests else "WARNING")
    
    # Run comprehensive test if quick tests mostly pass
    if passed_tests >= total_tests * 0.8:  # 80% pass rate
        print()
        print_status("Quick tests mostly passed - running comprehensive suite...", "INFO")
        comprehensive_passed = run_comprehensive_test()
    else:
        print_status("Quick tests failed - skipping comprehensive suite", "WARNING")
        comprehensive_passed = False
    
    # Final results
    elapsed_time = time.time() - start_time
    print()
    print("=" * 50)
    print_status(f"Validation completed in {elapsed_time:.1f} seconds", "INFO")
    
    if passed_tests == total_tests and comprehensive_passed:
        print_status("🎉 ALL TESTS PASSED - Torque vectoring system validated!", "SUCCESS")
        return 0
    elif passed_tests >= total_tests * 0.8:
        print_status("⚠️  Most tests passed - System mostly functional", "WARNING")
        return 1
    else:
        print_status("❌ Multiple test failures - System validation failed", "ERROR")
        return 2

if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print_status("Validation interrupted by user", "WARNING")
        sys.exit(130)
    except Exception as e:
        print_status(f"Validation script error: {str(e)}", "ERROR")
        sys.exit(1)