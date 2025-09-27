# ESP32 Integration Guide

This guide explains how to integrate ESP32 sensors with the Diablo FSW system for dynamic sensor data collection and observation matrix building.

## Overview

The ESP32 integration system provides:
- **Serial Communication**: Receives sensor data from ESP32 devices via USB/serial
- **Dynamic Sensor Handling**: Automatically detects and handles variable sensor counts
- **Observation Matrices**: Builds matrices for sensor fusion algorithms
- **Real-time Processing**: Processes sensor data in real-time with configurable timeouts

## Architecture

```
ESP32 Sensors → Serial/USB → ESP32SerialHandler → ObservationMatrixBuilder → Sensor Fusion
     ↓              ↓              ↓                      ↓                    ↓
[PT, IMU, etc.] [Binary/Text] [Parse & Validate] [Build H Matrix] [Kalman Filter]
```

## ESP32 Arduino Code Setup

Your ESP32 should send data in the `SampleRecord` format:

```cpp
#pragma pack(push, 1)
struct SampleRecord {
  uint32_t t_us;              // timestamp in microseconds
  uint8_t  channel;           // sensor channel (0-9)
  int32_t  volt_reader;       // raw ADC reading
  float    voltage;           // converted voltage
  uint32_t read_time_us;      // read time in microseconds
  float    samples_per_second; // calculated sample rate
  uint32_t sent_us;           // sent timestamp
};
#pragma pack(pop)
```

### Example Arduino Code

```cpp
// Send binary data (recommended for production)
Serial.write((uint8_t *)&rec, sizeof(rec));

// Or send text data (for debugging)
for (int i = 0; i < NUM_PTS; i++) {
  Serial.print(voltages[i], 6);
  if (i < NUM_PTS - 1) Serial.print(" ");
}
Serial.print("\r\n");
```

## C++ Integration

### 1. Basic Setup

```cpp
#include "comms/include/ESP32SerialHandler.hpp"
#include "nav/include/ObservationMatrix.hpp"

// Create ESP32 handler
auto esp32_handler = createESP32Handler("/dev/ttyUSB0", 115200);

// Create observation matrix builder
auto config = getDefaultObservationMatrixConfig();
config.max_data_age_ms = 1000.0;  // 1 second timeout
auto observation_builder = std::make_shared<ObservationMatrixBuilder>(config);

// Register callback for sensor data
esp32_handler->registerSensorCallback(
    [](uint8_t sensor_id, double voltage, uint64_t timestamp, double sample_rate) {
        std::cout << "Sensor " << static_cast<int>(sensor_id) 
                  << ": " << voltage << "V @ " << sample_rate << "Hz" << std::endl;
    }
);

// Start the handler
esp32_handler->start();
```

### 2. Dynamic Sensor Detection

```cpp
// Get currently active sensors
auto active_sensors = esp32_handler->getActiveSensors();
std::cout << "Active sensors: ";
for (uint8_t sensor_id : active_sensors) {
    std::cout << static_cast<int>(sensor_id) << " ";
}
std::cout << std::endl;

// Get recent data for specific sensors
auto recent_data = esp32_handler->getAllRecentSensorData(1000); // 1 second
for (const auto& pair : recent_data) {
    uint8_t sensor_id = pair.first;
    auto pt_message = pair.second;
    std::cout << "Sensor " << static_cast<int>(sensor_id) << " data available" << std::endl;
}
```

### 3. Building Observation Matrices

```cpp
// Get PT sensor data
auto recent_data = esp32_handler->getAllRecentSensorData(1000);
std::vector<std::shared_ptr<PTMessage>> pt_messages;
for (const auto& pair : recent_data) {
    pt_messages.push_back(pair.second);
}

// Add to observation builder
observation_builder->clear();
observation_builder->addPTSensors(pt_messages, true, true); // pressure & temperature

// Build observation matrix for engine state estimation
auto result = observation_builder->buildEngineStateObservationMatrix();

if (result.valid) {
    std::cout << "Observation matrix: " << result.observation_matrix.rows() 
              << " x " << result.observation_matrix.cols() << std::endl;
    std::cout << "Measurement vector: " << result.measurement_vector.size() << " elements" << std::endl;
    
    // Use in your Kalman filter or sensor fusion algorithm
    // H = result.observation_matrix
    // z = result.measurement_vector
    // R = result.measurement_covariance
}
```

### 4. Handling Variable Sensor Counts

The system automatically handles cases where only some sensors are present:

```cpp
// Example: Only sensors 1, 2, 6, 8, 10 are active
// The system will:
// 1. Only include active sensors in observation matrices
// 2. Set unused sensor fields to null/invalid
// 3. Adjust matrix dimensions accordingly

auto active_sensors = esp32_handler->getActiveSensors(); // [1, 2, 6, 8, 10]
auto result = observation_builder->buildEngineStateObservationMatrix();

// result.sensor_ids will contain [1, 2, 6, 8, 10]
// result.measurement_vector will have 5 elements (one per active sensor)
// result.observation_matrix will be 5 x state_size
```

## Configuration Options

### ESP32SerialHandler Configuration

```cpp
ESP32Config config;
config.device_path = "/dev/ttyUSB0";        // Serial device path
config.baud_rate = 115200;                  // Baud rate
config.max_buffer_size = 1024;              // Buffer size
config.timeout_ms = 100;                    // Timeout
config.enable_binary_mode = true;           // Binary vs text mode
config.max_sensors = 10;                    // Maximum sensor count
```

### ObservationMatrixBuilder Configuration

```cpp
ObservationMatrixConfig config;
config.max_data_age_ms = 1000.0;            // Data timeout
config.time_sync_tolerance_ms = 50.0;       // Time sync tolerance
config.enable_outlier_detection = true;     // Enable outlier detection
config.outlier_threshold_sigma = 3.0;       // Outlier threshold
config.enable_interpolation = false;        // Data interpolation
config.interpolation_window_ms = 100.0;     // Interpolation window
config.max_sensors_per_type = 10;           // Max sensors per type
```

## Running the Example

1. **Connect ESP32**: Connect your ESP32 to the Jetson/computer via USB
2. **Upload Arduino Code**: Upload your sensor reading code to the ESP32
3. **Run Integration Example**:

```bash
# Build the project
cd /path/to/diablo-fsw
mkdir build && cd build
cmake ..
make

# Run the example (replace /dev/ttyUSB0 with your device)
./engine_controller /dev/ttyUSB0
```

## Troubleshooting

### Common Issues

1. **Permission Denied**: Add user to dialout group
   ```bash
   sudo usermod -a -G dialout $USER
   # Then logout and login again
   ```

2. **Device Not Found**: Check device path
   ```bash
   ls /dev/ttyUSB*  # Linux
   ls /dev/cu.*     # macOS
   ```

3. **No Data Received**: Check baud rate and data format
   ```bash
   # Test with minicom or similar
   minicom -D /dev/ttyUSB0 -b 115200
   ```

4. **Binary Data Issues**: Try text mode for debugging
   ```cpp
   config.enable_binary_mode = false;
   ```

### Debug Output

Enable debug output to see what's happening:

```cpp
// The system will print:
// - Connected sensors
// - Received data
// - Observation matrix dimensions
// - Sensor statistics
```

## Integration with Sensor Fusion

Once you have observation matrices, integrate with your sensor fusion algorithms:

```cpp
// Example Kalman filter integration
void updateKalmanFilter(const ObservationMatrixResult& observation) {
    if (!observation.valid) return;
    
    // Predict step
    kalman_filter.predict(dt);
    
    // Update step with observation
    kalman_filter.update(
        observation.measurement_vector,      // z
        observation.observation_matrix,      // H
        observation.measurement_covariance   // R
    );
}
```

## Performance Considerations

- **Real-time Constraints**: The system is designed for real-time operation
- **Memory Usage**: Sensor data is buffered with configurable limits
- **CPU Usage**: Processing is optimized for minimal CPU overhead
- **Network Bandwidth**: Binary mode reduces bandwidth requirements

## Future Enhancements

- **Multiple ESP32 Support**: Connect multiple ESP32 devices simultaneously
- **Wireless Communication**: Support for WiFi/Bluetooth ESP32 communication
- **Advanced Filtering**: Additional data filtering and validation options
- **Calibration Integration**: Automatic sensor calibration and drift correction
