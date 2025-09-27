# PT Calibration Framework Guide

## Overview

This document provides a comprehensive guide to the Pressure Transducer (PT) Calibration Framework implemented for the Diablo FSW system. The framework implements the mathematical models described in the `PressureTransducerCalibrationFramework.tex` document, providing robust, adaptive, and mathematically rigorous calibration capabilities.

## Table of Contents

1. [Framework Architecture](#framework-architecture)
2. [Mathematical Models](#mathematical-models)
3. [Calibration Map Functions](#calibration-map-functions)
4. [Environmental Modeling](#environmental-modeling)
5. [Change Detection](#change-detection)
6. [EKF Integration](#ekf-integration)
7. [Usage Examples](#usage-examples)
8. [Configuration](#configuration)
9. [Validation and Quality Metrics](#validation-and-quality-metrics)
10. [Best Practices](#best-practices)

## Framework Architecture

The PT Calibration Framework consists of several interconnected components:

```
┌─────────────────────────────────────────────────────────────┐
│                 PT Calibration Framework                    │
├─────────────────────────────────────────────────────────────┤
│  CalibrationMapFunction  │  EnvironmentalState  │  Data     │
│  - Polynomial            │  - Temperature       │  Points   │
│  - Environmental Robust  │  - Humidity          │           │
│  - Adaptive Spline       │  - Vibration         │           │
│  - Physics-Informed NN   │  - Aging Factor      │           │
│                          │  - Mounting Torque   │           │
├─────────────────────────────────────────────────────────────┤
│  PTCalibrationFramework  │  Change Detection    │  EKF      │
│  - Bayesian Regression   │  - GLR Test          │  - State  │
│  - Total Least Squares   │  - CUSUM Test        │  - Covar. │
│  - Recursive LS          │  - Threshold Mgmt    │  - Update │
├─────────────────────────────────────────────────────────────┤
│  Calibration Tools       │  Validation          │  Monitor  │
│  - Session Management    │  - Quality Metrics   │  - Real-  │
│  - Data Collection       │  - Cross-Validation  │    time   │
│  - Report Generation     │  - Extrapolation     │    Status │
└─────────────────────────────────────────────────────────────┘
```

## Mathematical Models

### Complete Measurement Model

The framework implements the complete measurement model from the LaTeX document:

```
p_true = f(v; θ) + b(t) + ε_meas + ε_temp + ε_aging
```

Where:
- `v`: Transducer voltage
- `f(·; θ)`: Deterministic calibration map
- `b(t)`: Time-varying bias
- `ε_meas`: Measurement noise ~ N(0, σ_meas²)
- `ε_temp`: Temperature-induced noise ~ N(0, σ_temp²)
- `ε_aging`: Aging/drift noise ~ N(0, σ_aging²)

### Environmental State Vector

```
e = [T, H, V, A, M]ᵀ
```

Where:
- `T`: Temperature (°C)
- `H`: Humidity (%)
- `V`: Vibration level (normalized)
- `A`: Aging factor (time-dependent)
- `M`: Mounting torque factor

## Calibration Map Functions

### 1. Polynomial Calibration Map

Basic physically-informed polynomial model:

```cpp
f(v; θ) = θ₀ + θ₁v + θ₂v² + θ₃v³ + θ₄√v + θ₅log(1+v)
```

**Usage:**
```cpp
auto calibration_map = createCalibrationMap("polynomial");
auto framework = std::make_shared<PTCalibrationFramework>(calibration_map);
```

### 2. Environmental-Robust Calibration Map

Advanced model with environmental adaptation:

```cpp
f(v, e; θ) = Σₖ₌₀ⁿ θₖ φₖ(v, e)
```

With robust basis functions:
- `φ₀(v, e) = 1`
- `φ₁(v, e) = v`
- `φ₂(v, e) = v² + α₁Tv + α₂Hv`
- `φ₃(v, e) = v³ + β₁Tv² + β₂Vv`
- `φ₄(v, e) = √v + γ₁A·log(v)`
- `φ₅(v, e) = log(1+v) + δ₁T + δ₂H`

**Usage:**
```cpp
auto calibration_map = createCalibrationMap("environmental_robust");
auto framework = std::make_shared<PTCalibrationFramework>(calibration_map);
```

## Environmental Modeling

### Unified Environmental Variance Model

The framework implements adaptive variance modeling:

```cpp
σ_total²(v, e) = σ_base² + σ_env²(v, e) + σ_nonlinear²(v, e)
```

Where:
```cpp
σ_env²(v, e) = eᵀQ_env e + v²eᵀQ_interaction e
σ_nonlinear²(v, e) = α₁v⁴ + α₂||e||²v² + α₃||e||⁴
```

### Configuration

```cpp
EnvironmentalVarianceModel variance_model;
variance_model.base_variance = 1.0;
variance_model.env_variance_matrix = Eigen::MatrixXd::Identity(5, 5);
variance_model.interaction_matrix = Eigen::MatrixXd::Identity(5, 5);
variance_model.nonlinear_variance_alpha1 = 0.001;
variance_model.nonlinear_variance_alpha2 = 0.0001;
variance_model.nonlinear_variance_alpha3 = 0.00001;

framework.setEnvironmentalVarianceModel(variance_model);
```

## Change Detection

### Generalized Likelihood Ratio (GLR) Test

```cpp
GLRChangeDetector glr_detector(50, 0.05);  // window_size=50, false_alarm_rate=0.05

double glr_statistic = glr_detector.addMeasurement(
    voltage, pressure, environment, calibration_map, theta, covariance
);

if (glr_detector.isChangeDetected()) {
    // Trigger recalibration
}
```

### Cumulative Sum (CUSUM) Test

```cpp
CUSUMChangeDetector cusum_detector(5.0, 10);  // threshold=5.0, min_run_length=10

double cusum_statistic = cusum_detector.addMeasurement(
    voltage, pressure, environment, calibration_map, old_theta, new_theta
);

if (cusum_detector.isDriftDetected()) {
    // Detect gradual drift
}
```

## EKF Integration

### Adaptive EKF for Online Calibration

```cpp
PTCalibrationEKF ekf(calibration_map, 3);  // 3 physical states

// Initialize
ekf.initialize(initial_calibration, initial_environment, initial_physical_states);

// Predict step
ekf.predict(dt, &environmental_input);

// Update step
auto [innovation, innovation_variance] = ekf.update(voltage, pressure, environment);

// Get current state
auto state = ekf.getStateEstimate();
auto covariance = ekf.getStateCovariance();
```

### State Vector Structure

```
x = [x_phys, θ, e, b_residual]ᵀ
```

Where:
- `x_phys`: Physical system states
- `θ`: Calibration parameters
- `e`: Environmental state
- `b_residual`: Residual bias

## Usage Examples

### 1. Basic Calibration

```cpp
#include "PTCalibrationFramework.hpp"

// Create calibration framework
auto calibration_map = createCalibrationMap("environmental_robust");
auto framework = std::make_shared<PTCalibrationFramework>(calibration_map);

// Add calibration data
CalibrationDataPoint data_point;
data_point.voltage = 1.2;
data_point.reference_pressure = 500000.0;  // 500 kPa
data_point.reference_pressure_uncertainty = 100.0;  // 100 Pa
data_point.environment.temperature = 25.0;
data_point.environment.humidity = 50.0;
data_point.sensor_id = 0;
data_point.pt_location = static_cast<uint8_t>(PTLocation::PRESSURANT_TANK);

framework->addCalibrationData(data_point);

// Perform Bayesian calibration
Eigen::VectorXd prior_mean = Eigen::VectorXd::Zero(6);
Eigen::MatrixXd prior_cov = 1000.0 * Eigen::MatrixXd::Identity(6, 6);

auto calibration_result = framework->performBayesianCalibration(
    prior_mean, prior_cov, 1.0
);

// Predict pressure
auto [pressure, uncertainty] = framework->predictPressure(1.5, data_point.environment);
std::cout << "Predicted pressure: " << pressure << " Pa ± " << std::sqrt(uncertainty) << " Pa" << std::endl;
```

### 2. Real-Time Monitoring

```cpp
#include "PTCalibrationTool.hpp"

// Create calibration tool
PTCalibrationTool tool("environmental_robust");

// Start monitoring
auto& monitor = tool.getCalibrationMonitor();
monitor.startMonitoring(0, static_cast<uint8_t>(PTLocation::PRESSURANT_TANK));

// Add real-time measurements
PTMessage pt_message;
set_pt_measurement(pt_message, timestamp, 0, voltage, PTLocation::PRESSURANT_TANK);

EnvironmentalState environment;
environment.temperature = 25.0;
environment.humidity = 50.0;

monitor.addPTMeasurement(pt_message, reference_pressure, environment);

// Check status
auto status = monitor.getSensorStatus(0);
auto quality = monitor.getSensorQuality(0);
bool needs_recal = monitor.needsRecalibration(0);
```

### 3. Comprehensive Calibration Session

```cpp
// Create calibration procedure
auto procedure = tool.createCalibrationProcedure(
    "Pressurant Tank Calibration",
    0.0,        // Min pressure (Pa)
    1000000.0,  // Max pressure (Pa)
    15,         // Number of points
    true        // Include environmental variations
);

// Start calibration session
std::string session_id = tool.startCalibrationSession(
    0,  // sensor_id
    static_cast<uint8_t>(PTLocation::PRESSURANT_TANK),
    procedure
);

// Add calibration data points
for (double pressure : procedure.pressure_points) {
    for (int i = 0; i < 10; ++i) {
        double voltage = measure_voltage();  // Your measurement function
        EnvironmentalState env = get_environmental_state();  // Your environment function
        
        tool.addCalibrationDataPoint(session_id, voltage, pressure, env);
    }
}

// Complete calibration session
auto session = tool.completeCalibrationSession(session_id);

if (session.calibration_successful) {
    // Save calibration
    tool.saveCalibrationSession(session, "calibration_session.json");
    
    // Generate report
    std::string report = tool.generateCalibrationReport(session);
    std::cout << report << std::endl;
}
```

## Configuration

### Calibration Configuration File

Create a `pt_calibration_config.toml` file:

```toml
[calibration_map]
type = "environmental_robust"  # "polynomial" or "environmental_robust"

[environmental_variance]
base_variance = 1.0
env_variance_matrix = [
    [0.1, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.01, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.1, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.001, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.01]
]
interaction_matrix = [
    [0.001, 0.0, 0.0, 0.0, 0.0],
    [0.0, 0.0001, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.001, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.00001, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0001]
]
nonlinear_variance_alpha1 = 0.001
nonlinear_variance_alpha2 = 0.0001
nonlinear_variance_alpha3 = 0.00001

[change_detection]
glr_window_size = 50
glr_false_alarm_rate = 0.05
cusum_threshold = 5.0
cusum_min_run_length = 10

[ekf]
num_physical_states = 3
process_noise_variance = 0.1
measurement_noise_variance = 1.0
```

## Validation and Quality Metrics

### Quality Metrics

The framework provides comprehensive quality metrics:

```cpp
CalibrationQualityMetrics metrics = framework->computeQualityMetrics();

std::cout << "NRMSE: " << metrics.nrmse << std::endl;
std::cout << "95% Coverage: " << metrics.coverage_95 * 100 << "%" << std::endl;
std::cout << "Extrapolation Confidence: " << metrics.extrapolation_confidence * 100 << "%" << std::endl;
std::cout << "AIC: " << metrics.aic << std::endl;
std::cout << "BIC: " << metrics.bic << std::endl;
std::cout << "Condition Number: " << metrics.condition_number << std::endl;
```

### Validation Procedures

1. **Cross-Validation**: Use k-fold cross-validation to assess generalization
2. **Bootstrap Resampling**: Generate confidence intervals
3. **Extrapolation Testing**: Test performance outside calibration range
4. **Environmental Stress Testing**: Test across temperature/vibration ranges
5. **Long-term Drift Monitoring**: Monitor calibration stability over time

## Best Practices

### 1. Calibration Data Collection

- **Pressure Range**: Cover full operational range with adequate resolution
- **Environmental Conditions**: Include temperature, humidity, and vibration variations
- **Sample Size**: Minimum 10-20 samples per pressure point
- **Reference Standards**: Use traceable, high-accuracy reference instruments
- **Data Quality**: Ensure voltage stability and proper settling times

### 2. Model Selection

- **Polynomial**: Use for simple applications with minimal environmental variations
- **Environmental-Robust**: Use for applications with significant environmental changes
- **Adaptive Spline**: Use for maximum fidelity with complex nonlinear behavior
- **Physics-Informed NN**: Use for highly complex, multi-physics systems

### 3. Parameter Initialization

```cpp
// Good initialization for polynomial model
Eigen::VectorXd prior_mean(6);
prior_mean << 1000.0,    // offset (Pa)
              1000.0,    // linear coefficient (Pa/V)
              0.0,       // quadratic coefficient
              0.0,       // cubic coefficient
              0.0,       // sqrt coefficient
              0.0;       // log coefficient

Eigen::MatrixXd prior_cov = 1000.0 * Eigen::MatrixXd::Identity(6, 6);
```

### 4. Change Detection Tuning

- **GLR Window Size**: 30-100 samples depending on update rate
- **False Alarm Rate**: 0.01-0.05 depending on criticality
- **CUSUM Threshold**: 3-10 depending on sensitivity requirements
- **Update Frequency**: Balance between responsiveness and stability

### 5. EKF Tuning

```cpp
// Process noise tuning
Eigen::MatrixXd process_noise = Eigen::MatrixXd::Identity(state_size, state_size);
process_noise *= 0.1;  // Adjust based on system dynamics

// Measurement noise tuning
double measurement_noise = 1.0;  // Adjust based on sensor characteristics

ekf.setProcessNoiseCovariance(process_noise);
ekf.setMeasurementNoiseCovariance(measurement_noise);
```

### 6. Real-Time Implementation

- **Computational Efficiency**: Use sparse matrix operations for large systems
- **Memory Management**: Implement circular buffers for historical data
- **Parallel Processing**: Parallelize calibration across multiple sensors
- **Numerical Stability**: Monitor condition numbers and apply regularization

### 7. Integration with ESP32 System

```cpp
// ESP32 integration example
auto esp32_handler = createESP32SystemFromConfig("config/esp32_config.toml");
auto calibration_tool = std::make_shared<PTCalibrationTool>("environmental_robust");

// Register PT callback
esp32_handler->registerPTCallback([&](uint8_t sensor_id, double voltage, uint64_t timestamp, uint8_t pt_location) {
    // Create PT message
    PTMessage pt_message;
    set_pt_measurement(pt_message, timestamp, sensor_id, voltage, static_cast<PTLocation>(pt_location));
    
    // Get environmental state
    EnvironmentalState environment = get_current_environment();
    
    // Add to calibration monitor
    calibration_tool->getCalibrationMonitor().addPTMeasurement(pt_message, -1.0, environment);
});
```

## Troubleshooting

### Common Issues

1. **Poor Calibration Quality**
   - Check reference pressure accuracy
   - Verify voltage stability
   - Ensure adequate environmental coverage
   - Consider model complexity

2. **High Extrapolation Uncertainty**
   - Extend calibration range
   - Increase sample density at range boundaries
   - Use physics-informed constraints

3. **Frequent Change Detection Alarms**
   - Increase false alarm rate threshold
   - Improve environmental modeling
   - Check for sensor malfunctions

4. **EKF Divergence**
   - Reduce process noise
   - Improve initial state estimation
   - Check measurement model validity

### Performance Optimization

1. **Matrix Operations**: Use Eigen's optimized BLAS/LAPACK
2. **Memory Allocation**: Pre-allocate matrices for real-time operation
3. **Parallel Processing**: Use OpenMP for multi-sensor calibration
4. **Numerical Methods**: Use QR decomposition for least squares

## Conclusion

The PT Calibration Framework provides a comprehensive, mathematically rigorous solution for pressure transducer calibration in closed-loop control applications. By implementing Bayesian regression, Total Least Squares, change detection, and EKF integration, the framework ensures robust uncertainty quantification and adaptive online calibration capabilities.

For additional information, refer to:
- `PressureTransducerCalibrationFramework.tex` - Mathematical foundation
- `pt_calibration_example.cpp` - Implementation examples
- `PTCalibrationFramework.hpp` - API documentation
