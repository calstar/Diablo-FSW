# Liquid Engine Flight Software System

## Overview

This is a comprehensive flight software system for liquid rocket engine control, designed with a modular architecture that separates concerns into specialized subsystems. The system handles everything from sensor data processing to optimal control computation and valve actuation.

## System Architecture

```
FSW/
├── control/           # Engine control and valve management
├── comms/            # Communication protocols (Jetson + Ground Station)
├── calibration/      # Sensor and encoder calibration systems
├── nav/              # Navigation and sensor fusion
├── state/            # State machine and sequence control
└── src/              # Main application entry point
```

## Key Features

### 🎛️ **Control System** (`control/`)
- **Engine Control**: Main engine control logic with phase management
- **Valve Controller**: Motor-controlled and solenoid valve management
- **Gain Scheduling**: Adaptive control gains based on operating conditions
- **Optimal Controller**: MPC, LQR, and adaptive control algorithms

### 📡 **Communication System** (`comms/`)
- **Packet Protocol**: Jetson sensor data packet handling over Ethernet
- **Communication Protocol**: Ground station telemetry and command interface
- **Data Routing**: Intelligent routing of sensor data to appropriate subsystems

### 🔧 **Calibration System** (`calibration/`)
- **Sensor Calibration**: Bayesian calibration for PT, RTD, TC, IMU, GPS
- **Encoder Calibration**: Rotary encoder calibration and position mapping
- **Quality Assurance**: Calibration validation and drift detection

### 🧭 **Navigation System** (`nav/`)
- **EKF Navigation**: Extended Kalman Filter with dynamic state toggling
- **Sensor Fusion**: Multi-sensor data fusion with outlier rejection
- **State Integration**: Integration with engine state machine

### 📊 **State Management** (`state/`)
- **State Machine**: Engine phase management (pre-ignition → shutdown)
- **Sequence Control**: Automated sequences for ignition, shutdown, etc.
- **Safety Systems**: Abort conditions and fault detection

## Jetson Packet Protocol

The system receives sensor data from the Jetson in unified packets containing multiple sensor types:

### Packet Structure
```
[Header][Sensor1][Sensor2]...[SensorN]
```

### Supported Sensor Types
- **Pressure Transducers** (PT): Chamber, fuel inlet, ox inlet, coolant
- **Temperature Sensors** (RTD/TC): Wall temperature, fuel temp, exhaust temp
- **Load Cells**: Thrust measurement
- **IMU**: Accelerometer, gyroscope, magnetometer
- **GPS**: Position and velocity
- **Barometer**: Altitude sensing
- **Encoders**: Valve position feedback
- **Flow Meters**: Mass flow rate measurement

### Data Flow
1. **Packet Reception**: Jetson sends unified packets over Ethernet
2. **Deconstruction**: Packets are parsed and sensor data extracted
3. **Routing**: Data routed to appropriate subsystems based on type
4. **Processing**: Each subsystem processes relevant sensor data
5. **Control**: Optimal control commands computed
6. **Encoder Mapping**: Commands mapped to encoder values via calibration
7. **Transmission**: Commands sent back to Jetson for valve control

## Control Architecture

### Optimal Control Pipeline
1. **Sensor Data Fusion**: EKF combines all sensor data for state estimation
2. **State Machine Integration**: Navigation mode adapts to engine phase
3. **Gain Scheduling**: Control gains adapt to operating conditions
4. **Optimal Control**: MPC/LQR computes optimal valve commands
5. **Encoder Calibration**: Commands mapped to encoder counts
6. **Valve Control**: Commands sent to motor controllers via CAN bus

### Engine Phases
- **PRE_IGNITION**: System checks and purging
- **IGNITION**: Ignition sequence execution
- **STARTUP**: Startup transient control
- **STEADY_STATE**: Nominal operation
- **SHUTDOWN**: Controlled shutdown sequence
- **ABORT**: Emergency abort procedures

## Configuration

The system uses TOML-based configuration files:

```toml
# config/config_engine.toml
[engine_control]
control_frequency_hz = 100
safety_check_frequency_hz = 10

[valves.main_fuel]
type = "motor_controlled"
can_id = "0x101"
max_position = 1.0
default_rate_limit = 0.5

[sensors.pt_chamber]
type = "pressure_transducer"
calibration_file = "calibrations/pt_chamber.json"
measurement_range_max = 15e6  # Pa
```

## Calibration System

### Sensor Calibration
- **Bayesian Regression**: Implements the mathematical framework from the paper
- **Environmental Robustness**: Handles temperature, humidity, vibration effects
- **Uncertainty Quantification**: Full covariance propagation
- **Drift Detection**: GLR testing for calibration validation

### Encoder Calibration
- **Position Mapping**: Maps encoder counts to valve positions (0-1)
- **Nonlinear Correction**: Handles dead bands and backlash
- **Velocity Estimation**: Filtered velocity estimation
- **Health Monitoring**: Detects encoder faults and drift

## Building and Running

### Prerequisites
- CMake 3.20+
- C++20 compiler
- Eigen3
- CAN bus libraries (libcanard)
- Network libraries

### Build
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Run
```bash
# Start the engine controller
sudo ./engine_controller

# Or use the startup script
sudo ./scripts/start_engine_controller.sh
```

### Calibration
```bash
# Run calibration sequence
python3 scripts/calibration_sequence.py --interactive

# Or automated calibration
python3 scripts/calibration_sequence.py
```

## Development Workflow

### Adding New Sensors
1. Add sensor type to `PacketProtocol::SensorType`
2. Implement sensor-specific calibration in `SensorCalibration`
3. Add sensor handling in `SensorDataRouter`
4. Update navigation system if needed

### Adding New Control Algorithms
1. Implement algorithm in `OptimalController`
2. Add configuration parameters
3. Update gain scheduling if needed
4. Add tests and validation

### Modifying State Machine
1. Update `StateMachine::EngineState` enum
2. Add state transition logic
3. Update navigation mode integration
4. Add safety interlocks

## Testing and Validation

### Unit Tests
```bash
mkdir build && cd build
cmake -DENABLE_TESTS=ON ..
make test
```

### Integration Tests
- Hardware-in-the-loop testing with valve simulators
- Sensor data playback from recorded flights
- Monte Carlo validation of calibration algorithms

### Safety Validation
- Fault injection testing
- Emergency abort sequence validation
- Redundancy and failover testing

## Monitoring and Diagnostics

### Real-time Monitoring
- System health dashboard
- Sensor data visualization
- Control performance metrics
- Calibration quality indicators

### Logging
- Comprehensive event logging
- Sensor data recording
- Control command logging
- Performance metrics

### Diagnostics
- Automated fault detection
- Trend analysis
- Predictive maintenance
- Performance optimization

## Safety Features

### Redundancy
- Dual sensor measurements where critical
- Backup control algorithms
- Failover mechanisms
- Watchdog timers

### Fault Detection
- Sensor health monitoring
- Calibration drift detection
- Control performance monitoring
- Communication link monitoring

### Emergency Procedures
- Automatic abort conditions
- Safe shutdown sequences
- Valve emergency closure
- System isolation

## Performance Characteristics

### Timing Requirements
- **Control Loop**: 100 Hz (10 ms)
- **Navigation**: 50 Hz (20 ms)
- **Safety Monitoring**: 10 Hz (100 ms)
- **Telemetry**: 20 Hz (50 ms)

### Computational Load
- **Main Control**: ~5% CPU
- **Navigation EKF**: ~15% CPU
- **Sensor Processing**: ~10% CPU
- **Communication**: ~5% CPU

### Memory Usage
- **Static Allocation**: ~50 MB
- **Dynamic Buffers**: ~100 MB
- **Calibration Data**: ~10 MB
- **Log Buffers**: ~50 MB

## Future Enhancements

### Planned Features
- Machine learning-based control optimization
- Advanced fault diagnosis and prognosis
- Real-time performance optimization
- Enhanced sensor fusion algorithms

### Scalability
- Support for multiple engines
- Distributed control architecture
- Cloud-based data processing
- Advanced analytics and AI integration

## Contributing

1. Follow the modular architecture principles
2. Maintain comprehensive documentation
3. Add appropriate tests for new features
4. Ensure safety-critical code is thoroughly validated
5. Follow the established coding standards

## License

This software is proprietary and confidential. All rights reserved.

---

For detailed API documentation, see the individual header files in each subsystem directory.
