# Quick Start Guide

This guide will help you set up the distributed sensor system with a groundstation (laptop) and Jetson (remote device).

## Prerequisites

- **Elodin Database**: Install `elodin-db` on both systems
- **Network**: Both devices on same network
- **Dependencies**: CMake, tmux, C++17 compiler

## Step 1: Groundstation Setup (Laptop)

1. **Clone/Setup the system**:
   ```bash
   cd sensor_system
   source startup.sh
   ```

2. **Build the system**:
   ```bash
   ./build.sh
   ```

3. **Find your IP address**:
   ```bash
   hostname -I | awk '{print $1}'
   # Note this IP - you'll need it for the Jetson
   ```

4. **Start the groundstation**:
   ```bash
   ./groundstation/scripts/tmux_start_groundstation.sh test_db
   ```

5. **Verify it's working**:
   - You should see "Database is ready and listening on <IP>:2240"
   - The sensor viewer should start showing data (will be empty until Jetson connects)

## Step 2: Jetson Setup (Remote Device)

1. **Copy the system to Jetson**:
   ```bash
   # From your laptop
   scp -r sensor_system/ user@jetson-ip:/home/user/
   ```

2. **On the Jetson, setup environment**:
   ```bash
   cd sensor_system
   source startup.sh
   ```

3. **Build the system**:
   ```bash
   ./build.sh
   ```

4. **Update the groundstation IP**:
   ```bash
   # Edit config/config_jetson.toml
   nano config/config_jetson.toml
   
   # Change this line to your laptop's IP:
   groundstation_ip = "192.168.1.100"  # Replace with your laptop's IP
   ```

5. **Start the Jetson sensors**:
   ```bash
   ./shell/tmux_start_jetson_sensors.sh config/config_jetson.toml <laptop-ip>
   ```

## Step 3: Verify Everything Works

1. **On the groundstation**, you should see:
   - Database logs in left pane
   - Real-time sensor data in right pane
   - Data from all 6 sensor types streaming

2. **On the Jetson**, you should see:
   - Sensor data being generated and sent
   - Connection status to groundstation

## Troubleshooting

### Connection Issues

1. **Check network connectivity**:
   ```bash
   # From Jetson, ping groundstation
   ping <groundstation-ip>
   ```

2. **Check if port 2240 is open**:
   ```bash
   # On groundstation
   sudo ufw allow 2240
   # or
   sudo ufw disable  # for testing only
   ```

3. **Check if database is listening**:
   ```bash
   # On groundstation
   netstat -tlnp | grep 2240
   ```

### Build Issues

1. **Missing dependencies**:
   ```bash
   sudo apt update
   sudo apt install cmake build-essential tmux
   ```

2. **Elodin database not found**:
   ```bash
   # Make sure elodin-db is in PATH
   which elodin-db
   ```

## Stopping the System

- **Groundstation**: `tmux kill-session -t groundstation`
- **Jetson**: `tmux kill-session -t jetson_sensors`

## What You Should See

### Groundstation Output
```
[14:30:15.123] PT Sensor
  time_pt                : 1704123015.123
  pressure               : 101325.45 Pa
  temperature            : 25.67 °C
  time_monotonic         : 1704123015123456789

[14:30:15.133] IMU Sensor
  time_imu               : 1704123015.133
  accel                  : [0.123, -0.045, 9.812] m/s²
  gyro                   : [0.001, 0.002, -0.001] rad/s
  time_monotonic         : 1704123015133456789
```

### Jetson Output
```
PT: P=101325.45 Pa, T=25.67 C
IMU: Accel=[0.123, -0.045, 9.812], Gyro=[0.001, 0.002, -0.001]
TC: T=150.23 C, V=0.0067 V
```

## Next Steps

- Modify sensor data ranges in config files
- Add new sensor types
- Implement real sensor drivers
- Add data logging and analysis tools
