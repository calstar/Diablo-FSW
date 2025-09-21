#!/usr/bin/env python3
"""
Sensor System Configuration Generator
Generates environment-specific configuration files from base configuration
"""

import toml
import os
import argparse
import sys

# Get the script's directory (should be /config)
CONFIG_DIR = os.path.dirname(os.path.abspath(__file__))

# Base configuration file
BASE_CONFIG_FILE = os.path.join(CONFIG_DIR, "config_base.toml")

# Output configuration files
DEV_CONFIG_FILE = os.path.join(CONFIG_DIR, "config_dev.toml")
PROD_CONFIG_FILE = os.path.join(CONFIG_DIR, "config_prod.toml")
JETSON_CONFIG_FILE = os.path.join(CONFIG_DIR, "config_jetson_enhanced.toml")
GROUNDSTATION_CONFIG_FILE = os.path.join(CONFIG_DIR, "config_groundstation_enhanced.toml")

# Get ROOT_SENSOR_DIR dynamically from the environment (set by startup.sh)
ROOT_SENSOR_DIR = os.environ.get("ROOT_SENSOR_DIR")
if not ROOT_SENSOR_DIR:
    raise EnvironmentError("ROOT_SENSOR_DIR is not set! Ensure startup.sh is sourced first.")

# Development Profile - Local testing
DEV_PROFILE = {
    "sensor_config": {
        "enable_pt_sensors": True,
        "enable_tc_sensors": True,
        "enable_rtd_sensors": True,
        "enable_imu_sensors": True,
        "enable_barometer": True,
        "enable_gps": False,  # GPS disabled for development due to BufferUnderflow
        "sensor_data_rate_hz": 10,
    },
    "network": {
        "local_ip": "127.0.0.1",
        "groundstation_ip": "127.0.0.1",
        "publish_port": 2240,
        "buffer_size": 1024,
        "connection_timeout_ms": 5000,
    },
    "database": {
        "db_host": "127.0.0.1",
        "db_port": 2240,
        "auto_flush_interval_ms": 100,
        "max_buffer_size": 2048,
    },
    "logging": {
        "log_level": "debug",
        "enable_file_logging": True,
        "log_rotation_size_mb": 100,
    }
}

# Production Profile - Full system deployment
PROD_PROFILE = {
    "sensor_config": {
        "enable_pt_sensors": True,
        "enable_tc_sensors": True,
        "enable_rtd_sensors": True,
        "enable_imu_sensors": True,
        "enable_barometer": True,
        "enable_gps": True,  # GPS enabled for production
        "sensor_data_rate_hz": 50,  # Higher rate for production
    },
    "network": {
        "local_ip": "0.0.0.0",  # Listen on all interfaces
        "groundstation_ip": "192.168.1.100",
        "publish_port": 2240,
        "buffer_size": 4096,  # Larger buffer for production
        "connection_timeout_ms": 10000,
    },
    "database": {
        "db_host": "[::]:2240",  # IPv6 for production
        "db_port": 2240,
        "auto_flush_interval_ms": 50,  # More frequent flushing
        "max_buffer_size": 8192,  # Larger buffer
    },
    "logging": {
        "log_level": "info",
        "enable_file_logging": True,
        "log_rotation_size_mb": 500,
    }
}

# Jetson Profile - Remote sensor node
JETSON_PROFILE = {
    "sensor_config": {
        "enable_pt_sensors": True,
        "enable_tc_sensors": True,
        "enable_rtd_sensors": True,
        "enable_imu_sensors": True,
        "enable_barometer": True,
        "enable_gps": True,
        "sensor_data_rate_hz": 20,
    },
    "network": {
        "local_ip": "0.0.0.0",
        "groundstation_ip": "192.168.1.100",  # Will be overridden at runtime
        "publish_port": 2240,
        "buffer_size": 2048,
        "connection_timeout_ms": 15000,  # Longer timeout for remote
    },
    "database": {
        "db_host": "remote",  # Connect to remote database
        "db_port": 2240,
        "auto_flush_interval_ms": 25,  # Very frequent flushing for remote
        "max_buffer_size": 1024,
        "connection_retry_attempts": 5,
    },
    "logging": {
        "log_level": "info",
        "enable_file_logging": True,
        "log_rotation_size_mb": 200,
    }
}

# Groundstation Profile - Data collection and visualization
GROUNDSTATION_PROFILE = {
    "sensor_config": {
        "enable_pt_sensors": False,  # Groundstation doesn't generate sensors
        "enable_tc_sensors": False,
        "enable_rtd_sensors": False,
        "enable_imu_sensors": False,
        "enable_barometer": False,
        "enable_gps": False,
        "sensor_data_rate_hz": 0,
    },
    "network": {
        "local_ip": "0.0.0.0",  # Listen on all interfaces
        "groundstation_ip": "127.0.0.1",
        "publish_port": 2240,
        "buffer_size": 8192,  # Large buffer for data collection
        "connection_timeout_ms": 30000,
    },
    "database": {
        "db_host": "0.0.0.0:2240",  # Accept connections from anywhere
        "db_port": 2240,
        "auto_flush_interval_ms": 1000,  # Less frequent flushing for collection
        "max_buffer_size": 16384,  # Very large buffer
    },
    "visualization": {
        "enable_web_interface": True,
        "web_port": 8080,
        "refresh_rate_ms": 100,
        "max_data_points": 10000,
    },
    "logging": {
        "log_level": "info",
        "enable_file_logging": True,
        "log_rotation_size_mb": 1000,  # Large logs for groundstation
    }
}


def generate_config(base_config_file, output_file, profile, force_overwrite=False):
    """
    Generate a configuration file by applying profile overrides to the base config.
    
    Args:
        base_config_file (str): Path to the base configuration file
        output_file (str): Path to the output configuration file
        profile (dict): Profile overrides to apply
        force_overwrite (bool): Whether to overwrite existing files
        
    Returns:
        bool: True if config was generated/updated, False if skipped
    """
    
    # Check if output file exists and whether to skip or overwrite
    if os.path.exists(output_file):
        if not force_overwrite:
            print(f"Configuration file '{output_file}' already exists. Skipping (use --force to overwrite).")
            return False
        else:
            print(f"Overwriting existing configuration file: {output_file}")
    else:
        print(f"Creating new configuration file: {output_file}")
    
    # Validate base config file exists
    if not os.path.exists(base_config_file):
        raise FileNotFoundError(f"Base configuration file '{base_config_file}' not found!")
    
    try:
        # Load base config
        with open(base_config_file, "r") as f:
            base_config = toml.load(f)
        
        print(f"  Loaded base configuration from: {base_config_file}")
        
        # Apply profile overrides
        for section, updates in profile.items():
            if section not in base_config:
                print(f"  Adding new section: [{section}]")
                base_config[section] = {}
            else:
                print(f"  Updating section: [{section}]")
            
            for key, value in updates.items():
                if key in base_config[section]:
                    old_value = base_config[section][key]
                    print(f"    {key}: {old_value} -> {value}")
                else:
                    print(f"    {key}: (new) -> {value}")
                base_config[section][key] = value
        
        # Write the updated config to the output file
        with open(output_file, "w") as f:
            toml.dump(base_config, f)
        
        print(f"  ✅ Configuration written to {output_file}")
        return True
        
    except Exception as e:
        print(f"  ❌ Error generating config '{output_file}': {e}")
        raise

def validate_config_completeness(config_file):
    """
    Validate that a configuration file has all required sections and keys.
    
    Args:
        config_file (str): Path to configuration file to validate
        
    Returns:
        bool: True if validation passes, False otherwise
    """
    required_sections = [
        "sensor_config", "network", "database", "logging"
    ]
    
    try:
        with open(config_file, "r") as f:
            config = toml.load(f)
        
        missing_sections = []
        for section in required_sections:
            if section not in config:
                missing_sections.append(section)
        
        if missing_sections:
            print(f"  ❌ Configuration validation failed for {config_file}")
            print(f"    Missing sections: {missing_sections}")
            return False
        
        print(f"  ✅ Basic validation passed for {config_file}")
        return True
        
    except Exception as e:
        print(f"  ❌ Error validating config '{config_file}': {e}")
        return False

def main():
    parser = argparse.ArgumentParser(
        description="Generate environment-specific configuration files for sensor system",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python generate_configs.py                    # Generate configs only if they don't exist
  python generate_configs.py --force           # Force regenerate all configs
  python generate_configs.py --validate        # Only validate existing configs
  python generate_configs.py --force --validate # Regenerate and validate all configs
        """
    )
    
    parser.add_argument(
        "--force", 
        action="store_true", 
        help="Force overwrite existing configuration files"
    )
    
    parser.add_argument(
        "--validate", 
        action="store_true", 
        help="Validate generated configuration files"
    )
    
    parser.add_argument(
        "--verbose", "-v",
        action="store_true", 
        help="Enable verbose output"
    )
    
    args = parser.parse_args()
    
    print("=== Sensor System Configuration Generator ===")
    print(f"Root Sensor Directory: {ROOT_SENSOR_DIR}")
    print(f"Base Config: {BASE_CONFIG_FILE}")
    print()
    
    # Ensure the base configuration file exists
    if not os.path.exists(BASE_CONFIG_FILE):
        print(f"❌ Error: Base configuration file '{BASE_CONFIG_FILE}' not found!")
        sys.exit(1)
    
    # Configuration generation tasks
    configs_to_generate = [
        (DEV_CONFIG_FILE, DEV_PROFILE, "development"),
        (PROD_CONFIG_FILE, PROD_PROFILE, "production"),  
        (JETSON_CONFIG_FILE, JETSON_PROFILE, "jetson"),
        (GROUNDSTATION_CONFIG_FILE, GROUNDSTATION_PROFILE, "groundstation")
    ]
    
    generated_count = 0
    validation_passed = 0
    
    # Generate configurations
    for output_file, profile, env_name in configs_to_generate:
        print(f"--- {env_name.upper()} Configuration ---")
        try:
            was_generated = generate_config(BASE_CONFIG_FILE, output_file, profile, args.force)
            if was_generated:
                generated_count += 1
        except Exception as e:
            print(f"❌ Failed to generate {env_name} config: {e}")
            sys.exit(1)
        print()
    
    # Validate configurations if requested
    if args.validate:
        print("--- Configuration Validation ---")
        for output_file, _, env_name in configs_to_generate:
            if os.path.exists(output_file):
                if validate_config_completeness(output_file):
                    validation_passed += 1
            else:
                print(f"❌ Configuration file does not exist: {output_file}")
        print()
    
    # Summary
    print("=== Summary ===")
    if generated_count > 0:
        print(f"✅ Generated/updated {generated_count} configuration file(s)")
    else:
        print("ℹ️  No configuration files were generated (all existed and --force not used)")
    
    if args.validate:
        print(f"📊 {validation_passed}/{len(configs_to_generate)} configuration files passed validation")
        if validation_passed < len(configs_to_generate):
            print("⚠️  Some configuration files failed validation")
    
    print("\n🎉 Configuration generation complete!")
    print("To use a specific configuration:")
    print(f"  ./quick_start.sh with {DEV_CONFIG_FILE}      # Development")
    print(f"  ./quick_start.sh with {PROD_CONFIG_FILE}     # Production")  
    print(f"  ./quick_start.sh with {JETSON_CONFIG_FILE}   # Jetson Remote")
    print(f"  ./quick_start.sh with {GROUNDSTATION_CONFIG_FILE} # Groundstation")

if __name__ == "__main__":
    main()
