#!/usr/bin/env python3
"""
Automated Test Runner and Validator

This module provides a comprehensive automated test execution system
with pass/fail criteria, performance metrics, and detailed reporting
for the entire torque vectoring test suite.
"""

import rclpy
import sys
import os
import json
import time
import subprocess
import threading
from datetime import datetime
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import argparse

# Import all test suites
from signal_level_test_framework import SignalLevelTestFramework
from slip_detection_test_suite import SlipDetectionTestSuite
from torque_redistribution_test_suite import TorqueRedistributionTestSuite
from power_management_test_suite import PowerManagementTestSuite
from sensor_health_test_suite import SensorHealthTestSuite
from scenario_integration_test_suite import ScenarioIntegrationTestSuite

@dataclass
class TestSuiteResult:
    """Result from running a complete test suite"""
    suite_name: str
    total_tests: int
    passed_tests: int
    failed_tests: int
    execution_time: float
    success_rate: float
    test_results: List[Dict[str, Any]]
    errors: List[str]

@dataclass
class SystemValidationResult:
    """Complete system validation result"""
    validation_time: datetime
    total_test_suites: int
    overall_success_rate: float
    suite_results: List[TestSuiteResult]
    performance_metrics: Dict[str, Any]
    system_health: Dict[str, Any]
    recommendations: List[str]

class TorqueVectoringTestRunner:
    """Automated test runner for the complete torque vectoring system"""
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self.config = config or self._get_default_config()
        self.test_framework = None
        self.suite_results: List[TestSuiteResult] = []
        self.start_time = None
        
    def _get_default_config(self) -> Dict[str, Any]:
        """Get default test configuration"""
        return {
            'test_suites': {
                'slip_detection': True,
                'torque_redistribution': True,
                'power_management': True,
                'sensor_health': True,
                'scenario_integration': True
            },
            'performance_thresholds': {
                'min_success_rate': 90.0,  # %
                'max_execution_time': 300.0,  # seconds
                'max_memory_usage': 512.0,  # MB
                'min_response_time': 0.02  # seconds (50Hz)
            },
            'reporting': {
                'generate_html_report': True,
                'generate_json_report': True,
                'save_detailed_logs': True,
                'report_directory': 'test_reports'
            },
            'system_validation': {
                'check_ros_dependencies': True,
                'validate_node_startup': True,
                'monitor_system_resources': True
            }
        }
    
    def initialize_test_environment(self) -> bool:
        """Initialize the test environment and check dependencies"""
        
        print("Initializing torque vectoring test environment...")
        
        # Check ROS2 environment
        if not self._check_ros2_environment():
            print("ERROR: ROS2 environment not properly configured")
            return False
        
        # Initialize ROS2
        try:
            rclpy.init()
            print("✓ ROS2 initialized successfully")
        except Exception as e:
            print(f"ERROR: Failed to initialize ROS2: {e}")
            return False
        
        # Create test framework
        try:
            self.test_framework = SignalLevelTestFramework()
            print("✓ Test framework initialized")
        except Exception as e:
            print(f"ERROR: Failed to initialize test framework: {e}")
            return False
        
        # Create report directory
        os.makedirs(self.config['reporting']['report_directory'], exist_ok=True)
        print(f"✓ Report directory created: {self.config['reporting']['report_directory']}")
        
        return True
    
    def _check_ros2_environment(self) -> bool:
        """Check if ROS2 environment is properly configured"""
        
        required_env_vars = ['ROS_VERSION', 'ROS_DISTRO']
        for var in required_env_vars:
            if var not in os.environ:
                print(f"Missing environment variable: {var}")
                return False
        
        # Check if we're running ROS2
        if os.environ.get('ROS_VERSION') != '2':
            print(f"Expected ROS_VERSION=2, got {os.environ.get('ROS_VERSION')}")
            return False
        
        return True
    
    def run_slip_detection_tests(self) -> TestSuiteResult:
        """Run slip detection test suite"""
        
        print("\n" + "="*60)
        print("RUNNING SLIP DETECTION TESTS")
        print("="*60)
        
        start_time = time.time()
        errors = []
        
        try:
            # Create slip detection test suite
            slip_test_suite = SlipDetectionTestSuite(self.test_framework)
            
            # Run individual tests
            self.test_framework.run_test("No Slip Condition", 
                                       slip_test_suite.test_no_slip_condition)
            self.test_framework.run_test("Single Wheel Slip", 
                                       slip_test_suite.test_single_wheel_slip)
            self.test_framework.run_test("Turn Induced Slip", 
                                       slip_test_suite.test_turn_induced_slip)
            self.test_framework.run_test("Terrain Based Slip", 
                                       slip_test_suite.test_terrain_based_slip)
            self.test_framework.run_test("Noise Resilience", 
                                       slip_test_suite.test_noise_resilience)
            self.test_framework.run_test("Edge Cases", 
                                       slip_test_suite.test_edge_cases)
            
        except Exception as e:
            errors.append(f"Slip detection test execution error: {str(e)}")
        
        execution_time = time.time() - start_time
        
        # Get results
        suite_tests = [r for r in self.test_framework.test_results if "Slip" in r.test_name or "slip" in r.test_name.lower()]
        total_tests = len(suite_tests)
        passed_tests = sum(1 for r in suite_tests if r.passed)
        failed_tests = total_tests - passed_tests
        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        
        return TestSuiteResult(
            suite_name="Slip Detection",
            total_tests=total_tests,
            passed_tests=passed_tests,
            failed_tests=failed_tests,
            execution_time=execution_time,
            success_rate=success_rate,
            test_results=[{
                'name': r.test_name,
                'passed': r.passed,
                'execution_time': r.execution_time,
                'details': r.details,
                'error_message': r.error_message
            } for r in suite_tests],
            errors=errors
        )
    
    def run_torque_redistribution_tests(self) -> TestSuiteResult:
        """Run torque redistribution test suite"""
        
        print("\n" + "="*60)
        print("RUNNING TORQUE REDISTRIBUTION TESTS")
        print("="*60)
        
        start_time = time.time()
        errors = []
        
        try:
            # Clear previous results for this suite
            initial_count = len(self.test_framework.test_results)
            
            torque_test_suite = TorqueRedistributionTestSuite(self.test_framework)
            
            self.test_framework.run_test("Straight Line Equal Distribution", 
                                       torque_test_suite.test_straight_line_equal_distribution)
            self.test_framework.run_test("Left Turn Torque Boost", 
                                       torque_test_suite.test_left_turn_torque_boost)
            self.test_framework.run_test("Right Turn Torque Boost", 
                                       torque_test_suite.test_right_turn_torque_boost)
            self.test_framework.run_test("Slip Torque Reduction", 
                                       torque_test_suite.test_slip_torque_reduction)
            self.test_framework.run_test("Torque Redistribution from Slipping", 
                                       torque_test_suite.test_torque_redistribution_from_slipping)
            self.test_framework.run_test("Aggressive Maneuver Distribution", 
                                       torque_test_suite.test_aggressive_maneuver_torque_distribution)
            
        except Exception as e:
            errors.append(f"Torque redistribution test execution error: {str(e)}")
        
        execution_time = time.time() - start_time
        
        # Get results for this suite
        suite_tests = self.test_framework.test_results[initial_count:]
        total_tests = len(suite_tests)
        passed_tests = sum(1 for r in suite_tests if r.passed)
        failed_tests = total_tests - passed_tests
        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        
        return TestSuiteResult(
            suite_name="Torque Redistribution",
            total_tests=total_tests,
            passed_tests=passed_tests,
            failed_tests=failed_tests,
            execution_time=execution_time,
            success_rate=success_rate,
            test_results=[{
                'name': r.test_name,
                'passed': r.passed,
                'execution_time': r.execution_time,
                'details': r.details,
                'error_message': r.error_message
            } for r in suite_tests],
            errors=errors
        )
    
    def run_power_management_tests(self) -> TestSuiteResult:
        """Run power management test suite"""
        
        print("\n" + "="*60)
        print("RUNNING POWER MANAGEMENT TESTS")
        print("="*60)
        
        start_time = time.time()
        errors = []
        
        try:
            initial_count = len(self.test_framework.test_results)
            
            power_test_suite = PowerManagementTestSuite(self.test_framework)
            
            self.test_framework.run_test("Normal Power Consumption", 
                                       power_test_suite.test_normal_power_consumption)
            self.test_framework.run_test("High Load Power Limiting", 
                                       power_test_suite.test_high_load_power_limiting)
            self.test_framework.run_test("Power Spike Handling", 
                                       power_test_suite.test_power_spike_handling)
            self.test_framework.run_test("Power Efficiency Optimization", 
                                       power_test_suite.test_power_efficiency_optimization)
            self.test_framework.run_test("Emergency Power Management", 
                                       power_test_suite.test_emergency_power_management)
            
        except Exception as e:
            errors.append(f"Power management test execution error: {str(e)}")
        
        execution_time = time.time() - start_time
        
        suite_tests = self.test_framework.test_results[initial_count:]
        total_tests = len(suite_tests)
        passed_tests = sum(1 for r in suite_tests if r.passed)
        failed_tests = total_tests - passed_tests
        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        
        return TestSuiteResult(
            suite_name="Power Management",
            total_tests=total_tests,
            passed_tests=passed_tests,
            failed_tests=failed_tests,
            execution_time=execution_time,
            success_rate=success_rate,
            test_results=[{
                'name': r.test_name,
                'passed': r.passed,
                'execution_time': r.execution_time,
                'details': r.details,
                'error_message': r.error_message
            } for r in suite_tests],
            errors=errors
        )
    
    def run_sensor_health_tests(self) -> TestSuiteResult:
        """Run sensor health monitoring test suite"""
        
        print("\n" + "="*60)
        print("RUNNING SENSOR HEALTH MONITORING TESTS")
        print("="*60)
        
        start_time = time.time()
        errors = []
        
        try:
            initial_count = len(self.test_framework.test_results)
            
            sensor_test_suite = SensorHealthTestSuite(self.test_framework)
            
            self.test_framework.run_test("IMU Timeout Detection", 
                                       sensor_test_suite.test_imu_timeout_detection)
            self.test_framework.run_test("Joint States Timeout Detection", 
                                       sensor_test_suite.test_joint_states_timeout_detection)
            self.test_framework.run_test("Complete Sensor Failure", 
                                       sensor_test_suite.test_complete_sensor_failure)
            self.test_framework.run_test("Intermittent Sensor Failures", 
                                       sensor_test_suite.test_intermittent_sensor_failures)
            self.test_framework.run_test("Sensor Noise Tolerance", 
                                       sensor_test_suite.test_sensor_noise_tolerance)
            self.test_framework.run_test("Sensor Recovery", 
                                       sensor_test_suite.test_sensor_recovery)
            
        except Exception as e:
            errors.append(f"Sensor health test execution error: {str(e)}")
        
        execution_time = time.time() - start_time
        
        suite_tests = self.test_framework.test_results[initial_count:]
        total_tests = len(suite_tests)
        passed_tests = sum(1 for r in suite_tests if r.passed)
        failed_tests = total_tests - passed_tests
        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        
        return TestSuiteResult(
            suite_name="Sensor Health Monitoring",
            total_tests=total_tests,
            passed_tests=passed_tests,
            failed_tests=failed_tests,
            execution_time=execution_time,
            success_rate=success_rate,
            test_results=[{
                'name': r.test_name,
                'passed': r.passed,
                'execution_time': r.execution_time,
                'details': r.details,
                'error_message': r.error_message
            } for r in suite_tests],
            errors=errors
        )
    
    def run_scenario_integration_tests(self) -> TestSuiteResult:
        """Run scenario integration test suite"""
        
        print("\n" + "="*60)
        print("RUNNING SCENARIO INTEGRATION TESTS")
        print("="*60)
        
        start_time = time.time()
        errors = []
        
        try:
            initial_count = len(self.test_framework.test_results)
            
            scenario_test_suite = ScenarioIntegrationTestSuite(self.test_framework)
            
            self.test_framework.run_test("Highway Driving Scenario", 
                                       scenario_test_suite.test_highway_driving_scenario)
            self.test_framework.run_test("Urban Stop and Go Scenario", 
                                       scenario_test_suite.test_urban_stop_and_go_scenario)
            self.test_framework.run_test("Off-Road Terrain Scenario", 
                                       scenario_test_suite.test_off_road_terrain_scenario)
            self.test_framework.run_test("Emergency Maneuver Scenario", 
                                       scenario_test_suite.test_emergency_maneuver_scenario)
            self.test_framework.run_test("Long Duration Stability", 
                                       scenario_test_suite.test_long_duration_stability)
            
        except Exception as e:
            errors.append(f"Scenario integration test execution error: {str(e)}")
        
        execution_time = time.time() - start_time
        
        suite_tests = self.test_framework.test_results[initial_count:]
        total_tests = len(suite_tests)
        passed_tests = sum(1 for r in suite_tests if r.passed)
        failed_tests = total_tests - passed_tests
        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0
        
        return TestSuiteResult(
            suite_name="Scenario Integration",
            total_tests=total_tests,
            passed_tests=passed_tests,
            failed_tests=failed_tests,
            execution_time=execution_time,
            success_rate=success_rate,
            test_results=[{
                'name': r.test_name,
                'passed': r.passed,
                'execution_time': r.execution_time,
                'details': r.details,
                'error_message': r.error_message
            } for r in suite_tests],
            errors=errors
        )
    
    def run_all_test_suites(self) -> List[TestSuiteResult]:
        """Run all enabled test suites"""
        
        print("\n" + "🚀" + " "*20 + "STARTING COMPREHENSIVE TORQUE VECTORING TESTS" + " "*20 + "🚀")
        print("="*100)
        
        self.start_time = time.time()
        self.suite_results = []
        
        # Run each test suite if enabled
        if self.config['test_suites']['slip_detection']:
            result = self.run_slip_detection_tests()
            self.suite_results.append(result)
            self._print_suite_summary(result)
        
        if self.config['test_suites']['torque_redistribution']:
            result = self.run_torque_redistribution_tests()
            self.suite_results.append(result)
            self._print_suite_summary(result)
        
        if self.config['test_suites']['power_management']:
            result = self.run_power_management_tests()
            self.suite_results.append(result)
            self._print_suite_summary(result)
        
        if self.config['test_suites']['sensor_health']:
            result = self.run_sensor_health_tests()
            self.suite_results.append(result)
            self._print_suite_summary(result)
        
        if self.config['test_suites']['scenario_integration']:
            result = self.run_scenario_integration_tests()
            self.suite_results.append(result)
            self._print_suite_summary(result)
        
        return self.suite_results
    
    def _print_suite_summary(self, result: TestSuiteResult):
        """Print summary for a test suite"""
        
        status = "✅ PASSED" if result.success_rate >= self.config['performance_thresholds']['min_success_rate'] else "❌ FAILED"
        
        print(f"\n📊 {result.suite_name} Summary:")
        print(f"   Status: {status}")
        print(f"   Tests: {result.passed_tests}/{result.total_tests} passed ({result.success_rate:.1f}%)")
        print(f"   Time: {result.execution_time:.2f}s")
        
        if result.failed_tests > 0:
            print(f"   ⚠️ Failed Tests: {result.failed_tests}")
        
        if result.errors:
            print(f"   🚨 Errors: {len(result.errors)}")
    
    def generate_system_validation_result(self) -> SystemValidationResult:
        """Generate comprehensive system validation result"""
        
        total_execution_time = time.time() - self.start_time if self.start_time else 0
        
        # Calculate overall metrics
        total_tests = sum(suite.total_tests for suite in self.suite_results)
        total_passed = sum(suite.passed_tests for suite in self.suite_results)
        overall_success_rate = (total_passed / total_tests * 100) if total_tests > 0 else 0
        
        # Performance metrics
        performance_metrics = {
            'total_execution_time': total_execution_time,
            'average_test_time': total_execution_time / total_tests if total_tests > 0 else 0,
            'tests_per_second': total_tests / total_execution_time if total_execution_time > 0 else 0,
            'memory_usage_estimate': len(self.test_framework.debug_data_history) * 0.001,  # Rough estimate in MB
            'data_points_processed': len(self.test_framework.debug_data_history)
        }
        
        # System health assessment
        system_health = {
            'torque_vectoring_functional': any(
                suite.suite_name == "Torque Redistribution" and suite.success_rate > 80 
                for suite in self.suite_results
            ),
            'slip_detection_reliable': any(
                suite.suite_name == "Slip Detection" and suite.success_rate > 85 
                for suite in self.suite_results
            ),
            'power_management_effective': any(
                suite.suite_name == "Power Management" and suite.success_rate > 80 
                for suite in self.suite_results
            ),
            'sensor_monitoring_robust': any(
                suite.suite_name == "Sensor Health Monitoring" and suite.success_rate > 75 
                for suite in self.suite_results
            ),
            'integration_successful': any(
                suite.suite_name == "Scenario Integration" and suite.success_rate > 70 
                for suite in self.suite_results
            )
        }
        
        # Generate recommendations
        recommendations = self._generate_recommendations(system_health)
        
        return SystemValidationResult(
            validation_time=datetime.now(),
            total_test_suites=len(self.suite_results),
            overall_success_rate=overall_success_rate,
            suite_results=self.suite_results,
            performance_metrics=performance_metrics,
            system_health=system_health,
            recommendations=recommendations
        )
    
    def _generate_recommendations(self, system_health: Dict[str, bool]) -> List[str]:
        """Generate recommendations based on system health"""
        
        recommendations = []
        
        if not system_health['torque_vectoring_functional']:
            recommendations.append("Review torque redistribution algorithms - low success rate detected")
        
        if not system_health['slip_detection_reliable']:
            recommendations.append("Improve slip detection sensitivity and noise filtering")
        
        if not system_health['power_management_effective']:
            recommendations.append("Optimize power management algorithms and limits")
        
        if not system_health['sensor_monitoring_robust']:
            recommendations.append("Enhance sensor timeout detection and fallback mechanisms")
        
        if not system_health['integration_successful']:
            recommendations.append("Address integration issues in complex driving scenarios")
        
        # Performance recommendations
        total_execution_time = sum(suite.execution_time for suite in self.suite_results)
        if total_execution_time > self.config['performance_thresholds']['max_execution_time']:
            recommendations.append("Optimize test execution time - current runtime exceeds threshold")
        
        overall_success_rate = (
            sum(suite.passed_tests for suite in self.suite_results) / 
            sum(suite.total_tests for suite in self.suite_results) * 100
        ) if sum(suite.total_tests for suite in self.suite_results) > 0 else 0
        
        if overall_success_rate < self.config['performance_thresholds']['min_success_rate']:
            recommendations.append(f"Overall success rate ({overall_success_rate:.1f}%) below target threshold")
        
        if not recommendations:
            recommendations.append("System validation successful - all components functioning within acceptable parameters")
        
        return recommendations
    
    def generate_reports(self, validation_result: SystemValidationResult):
        """Generate comprehensive test reports"""
        
        report_dir = self.config['reporting']['report_directory']
        timestamp = validation_result.validation_time.strftime("%Y%m%d_%H%M%S")
        
        # Generate JSON report
        if self.config['reporting']['generate_json_report']:
            json_report = self._create_json_report(validation_result)
            json_filename = f"{report_dir}/torque_vectoring_test_report_{timestamp}.json"
            
            with open(json_filename, 'w') as f:
                json.dump(json_report, f, indent=2, default=str)
            
            print(f"📄 JSON report saved: {json_filename}")
        
        # Generate HTML report
        if self.config['reporting']['generate_html_report']:
            html_report = self._create_html_report(validation_result)
            html_filename = f"{report_dir}/torque_vectoring_test_report_{timestamp}.html"
            
            with open(html_filename, 'w') as f:
                f.write(html_report)
            
            print(f"🌐 HTML report saved: {html_filename}")
        
        # Save detailed logs
        if self.config['reporting']['save_detailed_logs']:
            log_filename = f"{report_dir}/torque_vectoring_detailed_log_{timestamp}.txt"
            self._save_detailed_log(validation_result, log_filename)
            print(f"📋 Detailed log saved: {log_filename}")
    
    def _create_json_report(self, validation_result: SystemValidationResult) -> Dict[str, Any]:
        """Create JSON format report"""
        
        return {
            'validation_summary': {
                'timestamp': validation_result.validation_time.isoformat(),
                'total_test_suites': validation_result.total_test_suites,
                'overall_success_rate': validation_result.overall_success_rate,
                'validation_status': 'PASSED' if validation_result.overall_success_rate >= 
                                   self.config['performance_thresholds']['min_success_rate'] else 'FAILED'
            },
            'test_suites': [
                {
                    'name': suite.suite_name,
                    'total_tests': suite.total_tests,
                    'passed_tests': suite.passed_tests,
                    'failed_tests': suite.failed_tests,
                    'success_rate': suite.success_rate,
                    'execution_time': suite.execution_time,
                    'test_results': suite.test_results,
                    'errors': suite.errors
                }
                for suite in validation_result.suite_results
            ],
            'performance_metrics': validation_result.performance_metrics,
            'system_health': validation_result.system_health,
            'recommendations': validation_result.recommendations,
            'configuration': self.config
        }
    
    def _create_html_report(self, validation_result: SystemValidationResult) -> str:
        """Create HTML format report"""
        
        html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Torque Vectoring System Test Report</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        .header {{ background-color: #f0f0f0; padding: 20px; border-radius: 5px; }}
        .summary {{ background-color: #e8f5e8; padding: 15px; margin: 20px 0; border-radius: 5px; }}
        .failed {{ background-color: #ffe8e8; }}
        .test-suite {{ margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }}
        .passed {{ color: green; }}
        .failed-text {{ color: red; }}
        table {{ border-collapse: collapse; width: 100%; }}
        th, td {{ border: 1px solid #ddd; padding: 8px; text-align: left; }}
        th {{ background-color: #f2f2f2; }}
        .recommendations {{ background-color: #fff3cd; padding: 15px; margin: 20px 0; border-radius: 5px; }}
    </style>
</head>
<body>
    <div class="header">
        <h1>🚗 Torque Vectoring System Test Report</h1>
        <p><strong>Validation Time:</strong> {validation_result.validation_time.strftime('%Y-%m-%d %H:%M:%S')}</p>
        <p><strong>Overall Success Rate:</strong> {validation_result.overall_success_rate:.1f}%</p>
    </div>
    
    <div class="summary {'failed' if validation_result.overall_success_rate < self.config['performance_thresholds']['min_success_rate'] else ''}">
        <h2>📊 Validation Summary</h2>
        <table>
            <tr><th>Metric</th><th>Value</th></tr>
            <tr><td>Total Test Suites</td><td>{validation_result.total_test_suites}</td></tr>
            <tr><td>Total Tests</td><td>{sum(suite.total_tests for suite in validation_result.suite_results)}</td></tr>
            <tr><td>Passed Tests</td><td>{sum(suite.passed_tests for suite in validation_result.suite_results)}</td></tr>
            <tr><td>Failed Tests</td><td>{sum(suite.failed_tests for suite in validation_result.suite_results)}</td></tr>
            <tr><td>Execution Time</td><td>{validation_result.performance_metrics['total_execution_time']:.2f}s</td></tr>
        </table>
    </div>
"""
        
        # Add test suite details
        for suite in validation_result.suite_results:
            status_class = "passed" if suite.success_rate >= 80 else "failed-text"
            html += f"""
    <div class="test-suite">
        <h3 class="{status_class}">{suite.suite_name}</h3>
        <p><strong>Success Rate:</strong> {suite.success_rate:.1f}% ({suite.passed_tests}/{suite.total_tests})</p>
        <p><strong>Execution Time:</strong> {suite.execution_time:.2f}s</p>
        
        <table>
            <tr><th>Test Name</th><th>Status</th><th>Time (s)</th></tr>
"""
            for test in suite.test_results:
                status = "✅ PASSED" if test['passed'] else "❌ FAILED"
                html += f"<tr><td>{test['name']}</td><td>{status}</td><td>{test['execution_time']:.2f}</td></tr>"
            
            html += "</table></div>"
        
        # Add recommendations
        html += f"""
    <div class="recommendations">
        <h2>💡 Recommendations</h2>
        <ul>
"""
        for rec in validation_result.recommendations:
            html += f"<li>{rec}</li>"
        
        html += """
        </ul>
    </div>
</body>
</html>"""
        
        return html
    
    def _save_detailed_log(self, validation_result: SystemValidationResult, filename: str):
        """Save detailed execution log"""
        
        with open(filename, 'w') as f:
            f.write("TORQUE VECTORING SYSTEM VALIDATION LOG\n")
            f.write("="*50 + "\n\n")
            f.write(f"Validation Time: {validation_result.validation_time}\n")
            f.write(f"Overall Success Rate: {validation_result.overall_success_rate:.1f}%\n\n")
            
            for suite in validation_result.suite_results:
                f.write(f"TEST SUITE: {suite.suite_name}\n")
                f.write("-" * 30 + "\n")
                f.write(f"Success Rate: {suite.success_rate:.1f}%\n")
                f.write(f"Execution Time: {suite.execution_time:.2f}s\n\n")
                
                for test in suite.test_results:
                    status = "PASSED" if test['passed'] else "FAILED"
                    f.write(f"  {test['name']}: {status} ({test['execution_time']:.2f}s)\n")
                    
                    if test['error_message']:
                        f.write(f"    Error: {test['error_message']}\n")
                
                f.write("\n")
    
    def cleanup(self):
        """Clean up test environment"""
        
        print("\n🧹 Cleaning up test environment...")
        
        if self.test_framework:
            try:
                self.test_framework.destroy_node()
                print("✓ Test framework destroyed")
            except Exception as e:
                print(f"Warning: Error destroying test framework: {e}")
        
        try:
            rclpy.shutdown()
            print("✓ ROS2 shutdown complete")
        except Exception as e:
            print(f"Warning: Error during ROS2 shutdown: {e}")
    
    def run_complete_validation(self) -> SystemValidationResult:
        """Run complete system validation"""
        
        try:
            # Initialize environment
            if not self.initialize_test_environment():
                raise RuntimeError("Failed to initialize test environment")
            
            # Run all test suites
            self.run_all_test_suites()
            
            # Generate validation result
            validation_result = self.generate_system_validation_result()
            
            # Generate reports
            self.generate_reports(validation_result)
            
            # Print final summary
            self._print_final_summary(validation_result)
            
            return validation_result
            
        finally:
            self.cleanup()
    
    def _print_final_summary(self, validation_result: SystemValidationResult):
        """Print final validation summary"""
        
        print("\n" + "🏁" + " "*20 + "TORQUE VECTORING SYSTEM VALIDATION COMPLETE" + " "*20 + "🏁")
        print("="*100)
        
        status = "✅ SYSTEM VALIDATED" if validation_result.overall_success_rate >= self.config['performance_thresholds']['min_success_rate'] else "❌ VALIDATION FAILED"
        print(f"\n🎯 Final Result: {status}")
        print(f"📈 Overall Success Rate: {validation_result.overall_success_rate:.1f}%")
        print(f"⏱️ Total Execution Time: {validation_result.performance_metrics['total_execution_time']:.2f}s")
        print(f"🧪 Total Tests Executed: {sum(suite.total_tests for suite in validation_result.suite_results)}")
        
        print(f"\n🔍 System Health Assessment:")
        for component, status in validation_result.system_health.items():
            status_icon = "✅" if status else "❌"
            print(f"   {status_icon} {component.replace('_', ' ').title()}")
        
        if validation_result.recommendations:
            print(f"\n💡 Key Recommendations:")
            for i, rec in enumerate(validation_result.recommendations[:3], 1):
                print(f"   {i}. {rec}")
        
        print("\n" + "="*100)

def main():
    """Main entry point for automated test execution"""
    
    parser = argparse.ArgumentParser(description='Automated Torque Vectoring System Test Runner')
    parser.add_argument('--config', type=str, help='Path to test configuration file')
    parser.add_argument('--suites', type=str, nargs='+', 
                       choices=['slip_detection', 'torque_redistribution', 'power_management', 
                               'sensor_health', 'scenario_integration'],
                       help='Specific test suites to run')
    parser.add_argument('--report-dir', type=str, default='test_reports',
                       help='Directory to save test reports')
    
    args = parser.parse_args()
    
    # Load configuration
    config = None
    if args.config and os.path.exists(args.config):
        with open(args.config, 'r') as f:
            config = json.load(f)
    
    # Create test runner
    test_runner = TorqueVectoringTestRunner(config)
    
    # Override report directory if specified
    if args.report_dir:
        test_runner.config['reporting']['report_directory'] = args.report_dir
    
    # Override test suites if specified
    if args.suites:
        for suite in test_runner.config['test_suites']:
            test_runner.config['test_suites'][suite] = suite in args.suites
    
    # Run validation
    try:
        validation_result = test_runner.run_complete_validation()
        
        # Exit with appropriate code
        exit_code = 0 if validation_result.overall_success_rate >= test_runner.config['performance_thresholds']['min_success_rate'] else 1
        sys.exit(exit_code)
        
    except KeyboardInterrupt:
        print("\n⚠️ Test execution interrupted by user")
        test_runner.cleanup()
        sys.exit(2)
    except Exception as e:
        print(f"\n🚨 Test execution failed: {e}")
        test_runner.cleanup()
        sys.exit(3)

if __name__ == '__main__':
    main()