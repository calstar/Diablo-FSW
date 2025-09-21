# Diablo-FSW Sensor System - Quick Start Guide

Get the Diablo-FSW telemetry system up and running in minutes!

## 🚀 **Recommended: One-Command Launch**

### **Quick Start (3-Pane Tmux)**
```bash
cd shell
./quick_start.sh [database_name]
```

**This automatically creates:**
- **Pane 1**: Elodin Database
- **Pane 2**: Sensor Generators  
- **Pane 3**: Data Visualizer

### **Clean Shutdown**
```bash
cd shell
./shutdown_system.sh
```

## 🔧 **Advanced: Manual Setup**

### 1. Build the System
```bash
mkdir build && cd build
cmake ..
make
```

### 2. Manual Startup
```bash
# Terminal 1: Start database
# On Linux/bash:
elodin-db run '[::]:2240' ~/.local/share/elodin/test_db
# On macOS/zsh (quote the address):
elodin-db run '[::]:2240' ~/.local/share/elodin/test_db

# Terminal 2: Start sensors
./scripts/fake_sensor_generator 127.0.0.1 2240

# Terminal 3: View data
python3 groundstation/scripts/sensor_data_viewer.py --host 127.0.0.1 --port 2240
```

### 3. Verify It's Working
- You should see continuous "[SENT]" messages from the sensor generator
- The viewer should show real-time plots of all sensor data
- Database logs should show "inserting vtable" messages (not BufferUnderflow errors)

## 🌐 Distributed Deployment (Ground Station + Remote)

### Ground Station (Laptop)
```bash
# Start ground station
./groundstation/scripts/start_groundstation.sh
```

### Remote Machine (Jetson)
```bash
# Build first
mkdir build && cd build && cmake .. && make

# Start remote sensors (replace with your ground station IP)
./scripts/start_remote_sensors.sh 192.168.1.100 2240
```

## 🔧 Troubleshooting

### No Data in Viewer?
1. Check database is running: `ps aux | grep elodin-db`
2. Check sensors are running: `ps aux | grep fake_sensor_generator`
3. Check database logs for errors
4. Verify packet IDs match between `dbConfig.hpp` and sensor generator

### Connection Issues?
1. Test connectivity: `telnet <ip> 2240`
2. Check firewall: `sudo ufw status`
3. Verify IP addresses in configuration files

### Build Issues?
1. Ensure C++20 compiler: `gcc --version`
2. Check CMake version: `cmake --version`
3. Install dependencies: `sudo apt install build-essential cmake`

## 📊 What You Should See

### Sensor Generator Output
```
✅ Connected to Elodin database at 127.0.0.1:2240
Generating database configuration...
Database configuration complete!
Starting fake sensor generators...
IMU: Accel=[0.1, 0.2, 9.8], Gyro=[0.01, 0.02, 0.03] [SENT]
PT: P=101325.5 Pa, T=25.1 C [SENT]
TC: T=150.2 C, V=0.005 V
...
```

### Database Logs
```
2025-09-04 16:20:14.835  INFO inserting vtable id=[1, 0]
2025-09-04 16:20:14.835  INFO inserting vtable id=[2, 0]
...
```

### Viewer Interface
- Real-time plots for all 7 sensor types
- Time cursor moving continuously
- Data points appearing in real-time
- Configurable time windows

## 🎯 Next Steps

1. **Customize Sensors**: Edit sensor frequencies and data ranges in `scripts/fake_sensor_generator.cpp`
2. **Add New Sensors**: Follow the pattern in `comms/include/` and `utl/dbConfig.hpp`
3. **Deploy to Production**: Use the systemd services in `DEPLOYMENT.md`
4. **Monitor Performance**: Use `scripts/health_check.sh` for automated monitoring

## 📚 Full Documentation

- `README.md` - Complete system overview
- `DEPLOYMENT.md` - Production deployment guide
- `config/` - Configuration files for different environments

## 🆘 Need Help?

Check the troubleshooting section in `README.md` or create an issue in the repository.