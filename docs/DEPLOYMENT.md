# Diablo-FSW Sensor System - Deployment Guide

This guide covers deploying the Diablo-FSW telemetry system to remote machines for distributed flight operations.

## 🚀 **Quick Deployment**

### **Automated Launch**
```bash
cd shell
./quick_start.sh [database_name]
```

### **Clean Shutdown**
```bash
cd shell
./shutdown_system.sh
```

### **Configuration Management**
```bash
cd config
python3 generate_configs.py --force --validate
```

## 📋 **Manual Deployment**

This guide covers manual deployment for distributed operation between a ground station (laptop) and remote sensors (Jetson or other Linux machines).

## Prerequisites

### Ground Station (Laptop)
- Ubuntu 20.04+ or similar Linux distribution
- Elodin database installed
- Python 3.8+ with required packages
- Network access to remote machines

### Remote Machine (Jetson/Remote)
- Ubuntu 20.04+ or similar Linux distribution
- C++20 compiler (GCC 10+ or Clang 12+)
- CMake 3.16+
- Network access to ground station

## Installation

### 1. Ground Station Setup

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd sensor_system
   ```

2. **Install Elodin database:**
   ```bash
   # Follow Elodin installation instructions
   # Ensure elodin-db is in PATH
   ```

3. **Install Python dependencies:**
   ```bash
   pip3 install matplotlib numpy pandas
   ```

4. **Configure network access:**
   ```bash
   # Edit groundstation/config/config_groundstation.toml
   # Set appropriate IP addresses and ports
   ```

### 2. Remote Machine Setup

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd sensor_system
   ```

2. **Install build dependencies:**
   ```bash
   sudo apt update
   sudo apt install build-essential cmake git
   ```

3. **Build the system:**
   ```bash
   mkdir build && cd build
   cmake ..
   make
   ```

4. **Configure for remote operation:**
   ```bash
   # Edit config/config_jetson.toml
   # Set ground station IP address
   ```

## Deployment Scripts

### Ground Station Startup Script

Create `groundstation/scripts/start_groundstation.sh`:
```bash
#!/bin/bash
set -e

echo "Starting Ground Station..."

# Kill any existing processes
pkill -f "elodin-db" || true
pkill -f "sensor_data_viewer" || true

# Clean up old database
rm -rf ~/.local/share/elodin/test_db*

# Start database
echo "Starting Elodin database..."
# Cross-platform compatible command:
elodin-db run '[::]:2240' ~/.local/share/elodin/test_db &
DB_PID=$!

# Wait for database to start
sleep 5

# Start data viewer
echo "Starting data viewer..."
python3 groundstation/scripts/sensor_data_viewer.py --host 0.0.0.0 --port 2240 &
VIEWER_PID=$!

echo "Ground station started!"
echo "Database PID: $DB_PID"
echo "Viewer PID: $VIEWER_PID"
echo "Access viewer at: http://localhost:8080"

# Wait for user to stop
echo "Press Ctrl+C to stop all services"
trap "kill $DB_PID $VIEWER_PID; exit" INT
wait
```

### Remote Machine Startup Script

Create `scripts/start_remote_sensors.sh`:
```bash
#!/bin/bash
set -e

GROUNDSTATION_IP=${1:-"192.168.1.100"}
PORT=${2:-"2240"}

echo "Starting Remote Sensors..."
echo "Connecting to ground station at $GROUNDSTATION_IP:$PORT"

# Kill any existing processes
pkill -f "fake_sensor_generator" || true

# Start sensor generator
echo "Starting sensor generator..."
./scripts/fake_sensor_generator_remote $GROUNDSTATION_IP $PORT &
SENSOR_PID=$!

echo "Remote sensors started!"
echo "Sensor PID: $SENSOR_PID"

# Wait for user to stop
echo "Press Ctrl+C to stop sensors"
trap "kill $SENSOR_PID; exit" INT
wait
```

## Network Configuration

### Firewall Setup

**Ground Station:**
```bash
# Allow incoming connections on port 2240
sudo ufw allow 2240/tcp
sudo ufw allow 8080/tcp  # For web viewer if implemented
```

**Remote Machine:**
```bash
# Allow outgoing connections
sudo ufw allow out 2240/tcp
```

### Network Testing

Test connectivity between machines:
```bash
# From remote machine to ground station
telnet <groundstation_ip> 2240

# From ground station, check if port is listening
netstat -tlnp | grep 2240
```

## Systemd Services (Optional)

For production deployment, create systemd services:

### Ground Station Service

Create `/etc/systemd/system/sensor-groundstation.service`:
```ini
[Unit]
Description=Sensor System Ground Station
After=network.target

[Service]
Type=simple
User=sensor
WorkingDirectory=/opt/sensor_system
ExecStart=/opt/sensor_system/groundstation/scripts/start_groundstation.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### Remote Sensor Service

Create `/etc/systemd/system/sensor-remote.service`:
```ini
[Unit]
Description=Sensor System Remote Sensors
After=network.target

[Service]
Type=simple
User=sensor
WorkingDirectory=/opt/sensor_system
ExecStart=/opt/sensor_system/scripts/start_remote_sensors.sh 192.168.1.100 2240
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

## Monitoring and Logging

### Log Configuration

Create log directories:
```bash
sudo mkdir -p /var/log/sensor_system
sudo chown sensor:sensor /var/log/sensor_system
```

### Health Check Script

Create `scripts/health_check.sh`:
```bash
#!/bin/bash

# Check if database is running
if ! pgrep -f "elodin-db" > /dev/null; then
    echo "ERROR: Database not running"
    exit 1
fi

# Check if sensors are running
if ! pgrep -f "fake_sensor_generator" > /dev/null; then
    echo "ERROR: Sensors not running"
    exit 1
fi

# Check network connectivity
if ! nc -z <groundstation_ip> 2240; then
    echo "ERROR: Cannot connect to ground station"
    exit 1
fi

echo "All systems operational"
```

## Troubleshooting

### Common Deployment Issues

1. **Connection Refused**
   - Check firewall settings
   - Verify IP addresses and ports
   - Ensure database is running on ground station

2. **Permission Denied**
   - Check file permissions on scripts
   - Ensure user has access to required directories
   - Verify sudo permissions if needed

3. **Build Failures**
   - Check C++20 compiler availability
   - Verify CMake version
   - Check for missing dependencies

4. **Data Not Appearing**
   - Check database logs for errors
   - Verify packet IDs match between components
   - Ensure flush_elodin() is being called

### Debug Commands

```bash
# Check running processes
ps aux | grep -E "(elodin|sensor|python)"

# Check network connections
netstat -tlnp | grep 2240

# Check system logs
journalctl -u sensor-groundstation -f
journalctl -u sensor-remote -f

# Test database connectivity
telnet <groundstation_ip> 2240
```

## Security Considerations

1. **Network Security**
   - Use VPN for remote connections
   - Implement proper authentication
   - Encrypt sensitive data transmission

2. **System Security**
   - Run services as non-root user
   - Implement proper file permissions
   - Regular security updates

3. **Data Security**
   - Backup database regularly
   - Implement data retention policies
   - Secure access to visualization tools

## Performance Tuning

### Database Optimization
- Adjust buffer sizes in Elodin configuration
- Monitor disk space usage
- Implement data archiving

### Network Optimization
- Use dedicated network interfaces
- Implement QoS for sensor data
- Monitor bandwidth usage

### System Optimization
- Adjust sensor frequencies based on requirements
- Monitor CPU and memory usage
- Implement resource limits


