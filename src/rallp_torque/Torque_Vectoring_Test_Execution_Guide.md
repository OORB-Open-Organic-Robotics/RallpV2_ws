# Torque Vectoring Test Execution Guide

## Quick Start

### 1. Prepare Environment
```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
cd ~/Desktop/OORB/rallp_v2
source install/setup.bash

# Verify torque vectoring node is available
ls src/rallp/src/torque_vectoring_node.py
```

### 2. Run Tests

#### Option A: Complete Test Suite (Recommended)
```bash
# Run all tests with default configuration
./src/rallp/src/run_torque_vectoring_tests.sh

# Run with verbose output
./src/rallp/src/run_torque_vectoring_tests.sh --verbose
```

#### Option B: Quick Test Subset
```bash
# Run reduced test set for faster validation
./src/rallp/src/run_torque_vectoring_tests.sh --quick
```

#### Option C: Specific Test Suite
```bash
# Run only slip detection tests
./src/rallp/src/run_torque_vectoring_tests.sh --suite slip_detection

# Run only torque redistribution tests  
./src/rallp/src/run_torque_vectoring_tests.sh --suite torque_redistribution

# Run only power management tests
./src/rallp/src/run_torque_vectoring_tests.sh --suite power_management

# Run only sensor health tests
./src/rallp/src/run_torque_vectoring_tests.sh --suite sensor_health

# Run only scenario integration tests
./src/rallp/src/run_torque_vectoring_tests.sh --suite scenario_integration
```

#### Option D: Custom Configuration
```bash
# Use custom test configuration
./src/rallp/src/run_torque_vectoring_tests.sh --config custom_config.json
```

### 3. Direct Python Execution
```bash
# Run automated test runner directly
cd src/rallp/src
python3 automated_test_runner.py

# With specific options
python3 automated_test_runner.py --suites slip_detection torque_redistribution --report-dir ../../../test_reports
```

## Test Results

### Exit Codes
- **0**: All tests passed - System validation successful
- **1**: Some tests failed - System validation failed  
- **2**: Test execution interrupted by user
- **3**: Test execution error occurred

### Report Locations
Test reports are generated in `test_reports/` directory:
- **HTML Report**: `torque_vectoring_test_report_YYYYMMDD_HHMMSS.html`
- **JSON Report**: `torque_vectoring_test_report_YYYYMMDD_HHMMSS.json`
- **Detailed Logs**: Individual test suite logs

### Opening Reports
```bash
# macOS
open test_reports/torque_vectoring_test_report_*.html

# Linux
xdg-open test_reports/torque_vectoring_test_report_*.html
```

## Test Configuration

### Default Configuration
The default configuration file is located at:
`src/rallp/config/torque_vectoring_test_config.json`

### Key Configuration Sections
- **test_suites**: Enable/disable specific test suites
- **test_parameters**: Algorithm-specific testing parameters
- **validation_criteria**: Pass/fail thresholds
- **performance_thresholds**: System performance requirements
- **simulation_settings**: Synthetic data generation settings

### Creating Custom Configurations
Copy the default configuration and modify as needed:
```bash
cp src/rallp/config/torque_vectoring_test_config.json my_config.json
# Edit my_config.json
./src/rallp/src/run_torque_vectoring_tests.sh --config my_config.json
```

## Individual Test Suites

### 1. Slip Detection Tests
**Purpose**: Validate slip detection algorithms across various terrain conditions
**Duration**: ~60 seconds
**Key Metrics**: Detection accuracy, response time, noise resilience

```bash
python3 slip_detection_test_suite.py
```

### 2. Torque Redistribution Tests  
**Purpose**: Test torque distribution logic during turns and slip conditions
**Duration**: ~90 seconds
**Key Metrics**: Distribution accuracy, turn assistance, slip compensation

```bash
python3 torque_redistribution_test_suite.py
```

### 3. Power Management Tests
**Purpose**: Validate power calculation, limiting, and safety systems
**Duration**: ~75 seconds  
**Key Metrics**: Power accuracy, limit enforcement, efficiency

```bash
python3 power_management_test_suite.py
```

### 4. Sensor Health Tests
**Purpose**: Test sensor timeout detection and fallback mechanisms
**Duration**: ~45 seconds
**Key Metrics**: Timeout detection, fallback activation, recovery time

```bash
python3 sensor_health_test_suite.py
```

### 5. Scenario Integration Tests
**Purpose**: End-to-end scenario testing for real-world driving conditions  
**Duration**: ~150 seconds
**Key Metrics**: System stability, scenario completion, performance consistency

```bash
python3 scenario_integration_test_suite.py
```

## Signal-Level Testing Framework

The signal-level testing framework provides synthetic data injection capabilities:

### TestSignal Generation
- **IMU Data**: Angular velocity, linear acceleration with configurable noise
- **Joint States**: Wheel positions, velocities, efforts
- **Command Velocity**: Linear and angular velocity commands
- **Terrain Effects**: Surface friction, slip conditions

### Validation Methods
- **Torque Validation**: Range checking, distribution accuracy
- **Power Validation**: Consumption limits, efficiency metrics  
- **Slip Validation**: Detection accuracy, threshold compliance
- **Response Time**: Algorithm latency measurement

## Troubleshooting

### Common Issues

#### ROS2 Environment Not Detected
```bash
# Solution: Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

#### Torque Vectoring Node Import Failed
```bash
# Check if node exists
ls src/rallp/src/torque_vectoring_node.py

# Verify Python path
export PYTHONPATH="$PWD/src/rallp/src:$PYTHONPATH"
```

#### Permission Denied on Test Script
```bash
# Make script executable
chmod +x src/rallp/src/run_torque_vectoring_tests.sh
```

#### Tests Fail with Timeout Errors
- Increase timeout values in configuration
- Check system performance and available resources
- Run tests with `--verbose` for detailed error information

### Debug Mode
For detailed debugging information:
```bash
# Enable verbose logging
./src/rallp/src/run_torque_vectoring_tests.sh --verbose

# Check individual test components
python3 -c "
import sys
sys.path.append('src/rallp/src')
from signal_level_test_framework import *
framework = SignalLevelTestFramework()
print('Framework initialized successfully')
"
```

## Performance Expectations

### System Requirements
- **CPU Usage**: < 80% during test execution
- **Memory Usage**: < 512 MB for complete test suite  
- **Test Duration**: 5-10 minutes for complete suite
- **Success Rate**: > 95% for production validation

### Validation Thresholds
- **Slip Detection**: 95% accuracy, 50ms response time
- **Torque Distribution**: 98% accuracy, 20ms response time  
- **Power Management**: 99% accuracy, 100ms response time
- **Sensor Health**: 97% accuracy, 20ms response time

## Continuous Integration

For automated testing in CI/CD pipelines:
```bash
# Non-interactive execution
./src/rallp/src/run_torque_vectoring_tests.sh --config ci_config.json > test_results.log 2>&1
echo "Test exit code: $?"
```

## Advanced Usage

### Custom Test Development
Extend the testing framework by inheriting from `SignalLevelTestFramework`:

```python
from signal_level_test_framework import SignalLevelTestFramework

class CustomTestSuite(SignalLevelTestFramework):
    def __init__(self):
        super().__init__()
        
    def run_custom_test(self):
        # Your custom test implementation
        pass
```

### Performance Profiling
Enable detailed performance metrics:
```bash
python3 automated_test_runner.py --enable-profiling --detailed-metrics
```

### Test Data Export
Export test data for further analysis:
```bash
python3 automated_test_runner.py --export-data --format csv
```