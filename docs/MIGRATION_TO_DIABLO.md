# Migration to Diablo-FSW Repository

This guide helps you migrate the sensor system to the Diablo-FSW repository.

## 📋 Pre-Migration Checklist

### **1. Repository Setup**
```bash
# Clone the Diablo-FSW repository
git clone git@github.com:calstar/Diablo-FSW.git
cd Diablo-FSW

# Create sensor system directory
mkdir -p telemetry/sensor_system
```

### **2. File Structure for Migration**
```
Diablo-FSW/
├── telemetry/
│   └── sensor_system/           # ← New sensor system location
│       ├── scripts/             # Sensor generators and utilities
│       ├── config/              # Configuration files and generator
│       ├── comms/               # Message definitions
│       ├── utl/                 # Database and socket utilities
│       ├── shell/               # Shell scripts and tmux launchers
│       ├── groundstation/       # Groundstation-specific scripts
│       ├── quick_start.sh       # Main launcher script
│       ├── shutdown_system.sh   # Clean shutdown script
│       └── README.md            # Updated documentation
```

## 🚀 Migration Steps

### **Step 1: Copy Core Files**
```bash
# From your current sensor_system directory:
cp -r scripts/ /path/to/Diablo-FSW/telemetry/sensor_system/
cp -r config/ /path/to/Diablo-FSW/telemetry/sensor_system/
cp -r comms/ /path/to/Diablo-FSW/telemetry/sensor_system/
cp -r utl/ /path/to/Diablo-FSW/telemetry/sensor_system/
cp -r shell/ /path/to/Diablo-FSW/telemetry/sensor_system/
cp -r groundstation/ /path/to/Diablo-FSW/telemetry/sensor_system/
cp -r external/ /path/to/Diablo-FSW/telemetry/sensor_system/
```

### **Step 2: Copy Scripts and Documentation**
```bash
cp quick_start.sh /path/to/Diablo-FSW/telemetry/sensor_system/
cp shutdown_system.sh /path/to/Diablo-FSW/telemetry/sensor_system/
cp startup.sh /path/to/Diablo-FSW/telemetry/sensor_system/
cp CMakeLists.txt /path/to/Diablo-FSW/telemetry/sensor_system/
cp build.sh /path/to/Diablo-FSW/telemetry/sensor_system/
cp README.md /path/to/Diablo-FSW/telemetry/sensor_system/
cp QUICK_START.md /path/to/Diablo-FSW/telemetry/sensor_system/
cp DEPLOYMENT.md /path/to/Diablo-FSW/telemetry/sensor_system/
```

### **Step 3: Update Path References**
After copying, you'll need to update these files to reflect the new paths:
- `startup.sh` - Update ROOT_SENSOR_DIR path
- `CMakeLists.txt` - Update any absolute paths
- All shell scripts - Update relative paths if needed

### **Step 4: Update Diablo-FSW Main README**
Add a section about the sensor system in the main Diablo-FSW README:

```markdown
## Telemetry System

The Diablo-FSW includes a comprehensive sensor telemetry system for real-time data collection and monitoring.

### Quick Start
```bash
cd telemetry/sensor_system
./quick_start.sh
```

See `telemetry/sensor_system/README.md` for detailed documentation.
```

## 🔧 Post-Migration Updates Needed

### **1. Update startup.sh**
```bash
# Change this line in startup.sh:
export ROOT_SENSOR_DIR="/path/to/Diablo-FSW/telemetry/sensor_system"
```

### **2. Update .gitignore**
Add to Diablo-FSW/.gitignore:
```
# Sensor system
telemetry/sensor_system/build/
telemetry/sensor_system/logs/
*.log
```

### **3. Integration with Main FSW**
- Consider integrating sensor data with main FSW telemetry
- Update build system to include sensor system
- Coordinate port usage with main FSW systems

## 🧪 Testing After Migration

### **1. Test Environment Setup**
```bash
cd telemetry/sensor_system
source startup.sh
```

### **2. Test System Launch**
```bash
./quick_start.sh diablo_test_db
```

### **3. Test Clean Shutdown**
```bash
./shutdown_system.sh
```

## 📚 Updated Documentation

All documentation has been updated to reflect:
- Diablo-FSW project context
- Flight software integration
- Rocket telemetry use cases
- Cross-platform compatibility (Linux/macOS)

## 🎯 Integration Benefits

- **Unified Telemetry**: Single system for all sensor data
- **Flight-Ready**: Tested packet handling and real-time streaming
- **Development Support**: Local testing without hardware
- **Distributed Operations**: Ground station + flight computer architecture
- **Cross-Platform**: Works on development (macOS) and flight (Linux) systems
