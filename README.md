# Sensor System

A distributed sensor data collection and visualization system built on the Elodin time-series database. This system simulates various sensor types (PT, TC, RTD, IMU, Barometer, GPS) and streams data to a central database for real-time monitoring and analysis.

## Features

- **Multiple Sensor Types**: Pressure/Temperature (PT), Thermocouple (TC), RTD, IMU (Accelerometer/Gyroscope), Barometer, GPS Position/Velocity
- **Real-time Data Streaming**: Continuous data generation with configurable frequencies
- **Stochastic Data**: Realistic sensor data with trends, drift, periodic variations, and noise
- **Distributed Architecture**: Ground station (laptop) + Remote sensors (Jetson)
- **Local Testing**: Single-machine mode for development and testing
- **Real-time Visualization**: Python-based data viewer with time-series plots
- **Monotonic Time**: Proper time synchronization using CLOCK_MONOTONIC

## Architecture

```
┌─────────────────┐    TCP/IP     ┌──────────────────┐
│   Ground Station │◄─────────────►│   Remote Sensors  │
│                 │    :2240      │                  │
│  ┌─────────────┐ │              │  ┌──────────────┐ │
│  │ Elodin DB   │ │              │  │ Sensor Gen.  │ │
│  └─────────────┘ │              │  └──────────────┘ │
│  ┌─────────────┐ │              └──────────────────┘
│  │ Data Viewer │ │
│  └─────────────┘ │
└─────────────────┘
```

## Quick Start

### Prerequisites

- C++20 compiler (GCC 10+ or Clang 12+)
- CMake 3.16+
- Python 3.8+
- Elodin database
- Linux (tested on Ubuntu 20.04+)

### Local Mode (Single Machine)

1. **Build the system:**
   ```bash
   cd sensor_system
   mkdir build && cd build
   cmake ..
   make
   ```

2. **Start the database:**
   ```bash
   elodin-db run [::]:2240 ~/.local/share/elodin/test_db
   ```

3. **Start sensor generators:**
   ```bash
   ./scripts/fake_sensor_generator 127.0.0.1 2240
   ```

4. **View data:**
   ```bash
   elodin
```

### Distributed Mode (Ground Station + Remote)

1. **On Ground Station (Laptop):**
   ```bash
   # Start database
   elodin-db run [::]:2240 ~/.local/share/elodin/test_db
   
   # Start data viewer
   elodin
      ```

2. **On Remote Machine (Jetson):**
   ```bash
   # Build and run sensor generator
   ./scripts/fake_sensor_generator_remote <groundstation_ip> 2240
   ```

## Sensor Types and Frequencies

| Sensor | Frequency | Data Type | Description |
|--------|-----------|-----------|-------------|
| IMU | 100 Hz | Accelerometer + Gyroscope | 3-axis motion sensors with vibration simulation |
| PT | 10 Hz | Pressure + Temperature | Atmospheric pressure and temperature with weather trends |
| TC | 5 Hz | Temperature + Voltage | Thermocouple temperature measurement |
| RTD | 2 Hz | Temperature + Resistance | Resistance temperature detector |
| Barometer | 20 Hz | Pressure + Altitude + Temperature | Barometric pressure and altitude |
| GPS Position | 1 Hz | Latitude + Longitude + Altitude | GPS coordinates with circular motion simulation |
| GPS Velocity | 1 Hz | North + East + Up velocity | GPS velocity components |

## Configuration

### Sensor Configuration
Edit `config/config_base.toml` for local settings or `config/config_jetson.toml` for remote deployment.

### Ground Station Configuration
Edit `groundstation/config/config_groundstation.toml` for ground station settings.

## Data Visualization

The Python viewer provides:
- Real-time time-series plots for all sensor types
- Configurable time windows
- Data export capabilities
- Multiple plot types (line, scatter, etc.)

## Development

### Project Structure
```
sensor_system/
├── comms/           # Message definitions and communication
├── config/          # Configuration files
├── external/        # External dependencies (MessageFactory)
├── groundstation/   # Ground station components
├── scripts/         # Sensor generators and utilities
├── shell/           # Shell scripts for orchestration
└── utl/            # Utilities (Elodin, TCP, database config)
```

### Building
```bash
mkdir build && cd build
cmake ..
make
```

### Adding New Sensors
1. Create message definition in `comms/include/`
2. Add vtable schema in `utl/dbConfig.hpp`
3. Implement generator function in `scripts/fake_sensor_generator.cpp`
4. Add to main loop and thread management

## Troubleshooting

### Common Issues

1. **"Couldn't find anything..." in viewer**
   - Ensure database is running
   - Check that `cppGenerateDBConfig()` is called
   - Verify packet IDs match between `dbConfig.hpp` and sensor generator

2. **"BufferUnderflow" errors in database logs**
   - Data is being sent but format is incorrect
   - Check message serialization and packet structure
   - Ensure `flush_elodin()` is called after each message

3. **Connection refused**
   - Verify database is running on correct port
   - Check firewall settings for distributed mode
   - Ensure correct IP addresses in configuration

4. **Compilation errors**
   - Ensure C++20 support
   - Check all dependencies are installed
   - Verify CMake configuration

### Debug Mode
Run with verbose output:
```bash
RUST_LOG=debug elodin-db run [::]:2240 ~/.local/share/elodin/test_db
```

## License

This project is based on the FSW (Flight Software) system and maintains compatibility with the Elodin database architecture.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## Deployment

For production deployment:
1. Set up proper systemd services for database and sensor generators
2. Configure log rotation
3. Set up monitoring and alerting
4. Use proper security configurations for network access
