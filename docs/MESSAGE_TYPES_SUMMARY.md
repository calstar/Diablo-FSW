# Flight Software Message Types Summary

## Overview

This document provides a comprehensive overview of all message types used in the liquid engine flight software system for the Elodin database. The system includes both legacy sensor messages and new flight software system messages with enhanced calibration and control data.

## Message ID Allocation

### Legacy Sensor Messages (0x01-0x07)
- **0x01**: IMU (Inertial Measurement Unit)
- **0x02**: PT (Pressure Transducer) - Basic
- **0x03**: TC (Thermocouple)
- **0x04**: RTD (Resistance Temperature Detector)
- **0x05**: Barometer
- **0x06**: GPS Position
- **0x07**: GPS Velocity

### Flight Software System Messages (0x10-0x15)
- **0x10**: Engine Control
- **0x11**: Valve Control
- **0x12**: Enhanced PT (Pressure Transducer with Calibration)
- **0x13**: Navigation (EKF State)
- **0x14**: Calibration Status
- **0x15**: System Health

### Reserved for Future Expansion (0x16-0xFF)
- **0x20-0x2F**: Additional Pressure Transducer Types
- **0x30-0x3F**: Additional Temperature Sensor Types
- **0x40-0x4F**: Load Cell and Force Sensors
- **0x50-0x5F**: Flow Meter Types
- **0x60-0x6F**: Encoder Types
- **0x70-0x7F**: Additional Navigation Messages
- **0x80-0x8F**: Communication Protocol Messages
- **0x90-0x9F**: Ground Station Messages
- **0xA0-0xAF**: Emergency and Safety Messages

## Message Type Details

### 1. Engine Control Message (0x10)

**Purpose**: Contains complete engine control state and performance metrics

**Fields**:
- `timestamp` (double): System timestamp in seconds
- `engine_phase` (uint8): Current engine phase (pre-ignition, startup, etc.)
- `thrust_demand` (double): Commanded thrust in Newtons
- `thrust_actual` (double): Actual thrust in Newtons
- `chamber_pressure` (double): Chamber pressure in Pascals
- `mixture_ratio_demand` (double): Commanded O/F ratio
- `mixture_ratio_actual` (double): Actual O/F ratio
- `fuel_valve_position` (double): Fuel valve position (0-1)
- `ox_valve_position` (double): Oxidizer valve position (0-1)
- `fuel_flow_rate` (double): Fuel mass flow rate in kg/s
- `ox_flow_rate` (double): Oxidizer mass flow rate in kg/s
- `specific_impulse` (double): Specific impulse in seconds
- `efficiency` (double): Overall efficiency (0-1)
- `ignition_confirmed` (bool): Ignition status
- `all_systems_go` (bool): System health status
- `time_monotonic` (uint64): Monotonic timestamp in nanoseconds

### 2. Valve Control Message (0x11)

**Purpose**: Individual valve control state and feedback

**Fields**:
- `timestamp` (double): System timestamp in seconds
- `valve_id` (uint8): Valve identifier
- `valve_type` (uint8): Valve type (motor/solenoid)
- `commanded_position` (double): Commanded position (0-1)
- `actual_position` (double): Actual position from encoder (0-1)
- `position_error` (double): Position error
- `velocity` (double): Current velocity in 1/s
- `current` (double): Motor current in Amperes
- `temperature` (double): Motor temperature in °C
- `rate_limit` (double): Position rate limit in 1/s
- `emergency_close` (bool): Emergency close flag
- `fault_detected` (bool): Fault status
- `valve_state` (uint8): Valve state enum
- `command_confidence` (double): Command confidence (0-1)
- `time_monotonic` (uint64): Monotonic timestamp in nanoseconds

### 3. Enhanced PT Message (0x12)

**Purpose**: Pressure transducer data with comprehensive calibration information

**Fields**:
- `timestamp` (double): System timestamp in seconds
- `sensor_id` (uint8): PT sensor identifier
- `raw_voltage` (double): Raw voltage reading in Volts
- `pressure` (double): Calibrated pressure in Pascals
- `pressure_uncertainty` (double): Measurement uncertainty in Pascals
- `temperature` (double): Sensor temperature in °C
- `calibration_quality` (double): Calibration quality (0-1)
- `calibration_valid` (bool): Calibration validity
- `drift_detected` (double): Drift detection flag
- `sensor_health` (uint8): Sensor health status
- `environmental_factor` (double): Environmental correction factor
- `time_monotonic` (uint64): Monotonic timestamp in nanoseconds

### 4. Navigation Message (0x13)

**Purpose**: Complete EKF navigation state with engine integration

**Fields**:
- `timestamp` (double): System timestamp in seconds
- `position_x`, `position_y`, `position_z` (double): Position in meters
- `velocity_x`, `velocity_y`, `velocity_z` (double): Velocity in m/s
- `attitude_qw`, `attitude_qx`, `attitude_qy`, `attitude_qz` (double): Quaternion attitude
- `angular_velocity_x`, `angular_velocity_y`, `angular_velocity_z` (double): Angular velocity in rad/s
- `acceleration_x`, `acceleration_y`, `acceleration_z` (double): Acceleration in m/s²
- `engine_thrust` (double): Engine thrust in Newtons
- `engine_mass` (double): Vehicle mass in kg
- `navigation_quality` (double): Navigation quality (0-1)
- `navigation_mode` (uint8): Navigation mode enum
- `navigation_valid` (bool): Navigation validity
- `time_monotonic` (uint64): Monotonic timestamp in nanoseconds

### 5. Calibration Message (0x14)

**Purpose**: Calibration status and quality metrics for all sensors

**Fields**:
- `timestamp` (double): System timestamp in seconds
- `sensor_id` (uint8): Sensor identifier
- `sensor_type` (uint8): Sensor type enum
- `calibration_status` (uint8): Calibration status enum
- `calibration_quality` (double): Overall calibration quality (0-1)
- `rmse` (double): Root mean square error
- `nrmse` (double): Normalized RMSE
- `coverage_95` (double): 95% confidence interval coverage
- `extrapolation_confidence` (double): Extrapolation confidence (0-1)
- `num_calibration_points` (uint32): Number of calibration points
- `drift_detected` (double): Drift detection flag
- `calibration_valid` (bool): Calibration validity
- `last_calibration_time` (double): Last calibration timestamp
- `sensor_health` (uint8): Sensor health status
- `time_monotonic` (uint64): Monotonic timestamp in nanoseconds

### 6. System Health Message (0x15)

**Purpose**: Overall system health and performance metrics

**Fields**:
- `timestamp` (double): System timestamp in seconds
- `system_status` (uint8): Overall system status
- `system_health` (double): Overall system health (0-1)
- `active_faults` (uint32): Number of active faults
- `total_faults` (uint32): Total number of faults
- `cpu_usage` (double): CPU usage percentage
- `memory_usage` (double): Memory usage percentage
- `network_quality` (double): Network communication quality (0-1)
- `control_performance` (double): Control system performance (0-1)
- `navigation_accuracy` (double): Navigation accuracy in meters
- `calibration_quality` (double): Overall calibration quality (0-1)
- `emergency_status` (uint8): Emergency system status
- `safety_systems_ok` (bool): Safety systems status
- `communication_ok` (bool): Communication systems status
- `time_monotonic` (uint64): Monotonic timestamp in nanoseconds

## Sensor Type Enums

### Pressure Transducer Types
- **PT_CHAMBER**: Chamber pressure transducer
- **PT_FUEL_INLET**: Fuel inlet pressure transducer
- **PT_OX_INLET**: Oxidizer inlet pressure transducer
- **PT_COOLANT_INLET**: Coolant inlet pressure transducer
- **PT_IGNITER**: Igniter pressure transducer

### Temperature Sensor Types
- **RTD_CHAMBER_WALL**: Chamber wall RTD
- **RTD_FUEL_TEMP**: Fuel temperature RTD
- **RTD_OX_TEMP**: Oxidizer temperature RTD
- **RTD_COOLANT_TEMP**: Coolant temperature RTD
- **TC_EXHAUST**: Exhaust gas thermocouple
- **TC_CHAMBER**: Chamber gas thermocouple
- **TC_COOLANT**: Coolant thermocouple

### Force and Flow Sensors
- **THRUST_LOAD_CELL**: Main thrust load cell
- **GIMBAL_LOAD_CELL**: Gimbal load cell
- **FUEL_FLOW_METER**: Fuel flow meter
- **OX_FLOW_METER**: Oxidizer flow meter
- **COOLANT_FLOW_METER**: Coolant flow meter

### Encoder Types
- **FUEL_VALVE_ENCODER**: Fuel valve encoder
- **OX_VALVE_ENCODER**: Oxidizer valve encoder
- **GIMBAL_X_ENCODER**: Gimbal X-axis encoder
- **GIMBAL_Y_ENCODER**: Gimbal Y-axis encoder

## Usage Examples

### Writing Engine Control Data
```cpp
#include "comms/include/EngineControlMessage.hpp"

EngineControlMessage msg;
set_engine_control_measurement(msg, 
    current_time,           // timestamp
    ENGINE_PHASE_STEADY,    // engine_phase
    5000.0,                // thrust_demand
    4980.0,                // thrust_actual
    2.5e6,                 // chamber_pressure
    6.2,                   // mixture_ratio_demand
    6.18,                  // mixture_ratio_actual
    0.75,                  // fuel_valve_position
    0.73,                  // ox_valve_position
    2.1,                   // fuel_flow_rate
    13.0,                  // ox_flow_rate
    285.0,                 // specific_impulse
    0.92,                  // efficiency
    true,                  // ignition_confirmed
    true,                  // all_systems_go
    time_monotonic         // time_monotonic
);

std::array<uint8_t, 2> packet_id = {0x10, 0x00};
write_to_elodindb(packet_id, msg);
```

### Writing Enhanced PT Data
```cpp
#include "comms/include/PTMessage.hpp"

PTMessage pt_msg;
set_pt_measurement(pt_msg,
    current_time,           // timestamp
    1,                     // sensor_id (chamber PT)
    2.45,                  // raw_voltage
    2.5e6,                 // pressure
    100.0,                 // pressure_uncertainty
    850.0,                 // temperature
    0.95,                  // calibration_quality
    true,                  // calibration_valid
    0.0,                   // drift_detected
    0,                     // sensor_health
    1.02,                  // environmental_factor
    time_monotonic         // time_monotonic
);

std::array<uint8_t, 2> packet_id = {0x12, 0x00};
write_to_elodindb(packet_id, pt_msg);
```

### Writing Navigation Data
```cpp
#include "comms/include/NavigationMessage.hpp"

NavigationMessage nav_msg;
set_navigation_measurement(nav_msg,
    current_time,           // timestamp
    0.0, 0.0, 100.0,       // position_x, position_y, position_z
    10.0, 0.5, 50.0,       // velocity_x, velocity_y, velocity_z
    1.0, 0.0, 0.0, 0.0,    // attitude quaternion
    0.01, 0.02, 0.0,       // angular_velocity
    0.0, 0.0, 9.81,        // acceleration
    5000.0,                // engine_thrust
    1000.0,                // engine_mass
    0.95,                  // navigation_quality
    NAV_MODE_GPS_AIDED,    // navigation_mode
    true,                  // navigation_valid
    time_monotonic         // time_monotonic
);

std::array<uint8_t, 2> packet_id = {0x13, 0x00};
write_to_elodindb(packet_id, nav_msg);
```

## Database Integration

All message types are automatically registered with the Elodin database through the `cppGenerateDBConfig()` function in `utl/dbConfig.hpp`. The database schema is generated at runtime and includes:

1. **VTable definitions** for each message type
2. **Component names** for all fields
3. **Entity names** for message identification
4. **Proper data types** and field sizes
5. **Indexing** for efficient querying

## Performance Considerations

### Message Frequency
- **Control Messages**: 100 Hz (10 ms)
- **Navigation Messages**: 50 Hz (20 ms)
- **Sensor Messages**: 10-100 Hz depending on sensor type
- **Health Messages**: 10 Hz (100 ms)
- **Calibration Messages**: 1 Hz (1000 ms)

### Data Rates
- **Engine Control**: ~200 bytes/message × 100 Hz = 20 KB/s
- **Navigation**: ~200 bytes/message × 50 Hz = 10 KB/s
- **Sensor Data**: ~100 bytes/message × 50 Hz = 5 KB/s per sensor
- **Total System**: ~100-200 KB/s depending on sensor count

### Storage Requirements
- **1 minute of data**: ~6-12 MB
- **1 hour of data**: ~360-720 MB
- **24 hours of data**: ~8.6-17.3 GB

## Future Extensions

The message system is designed for easy extension:

1. **New sensor types** can be added with unique message IDs
2. **Additional fields** can be added to existing messages
3. **Custom message types** can be created for specialized applications
4. **Message versioning** can be implemented for backward compatibility

## Troubleshooting

### Common Issues
1. **Message ID conflicts**: Ensure unique IDs for all message types
2. **Field size mismatches**: Verify field sizes match database schema
3. **Data type mismatches**: Use correct data types for all fields
4. **Timestamp synchronization**: Ensure all messages use consistent timestamps

### Debugging
- Use `generateTestMessage*()` functions for testing
- Check database logs for message parsing errors
- Verify message field counts and types
- Monitor system performance during high-frequency message writing

---

This message system provides a comprehensive foundation for the liquid engine flight software, enabling robust data logging, real-time monitoring, and post-flight analysis with full calibration and uncertainty information.
