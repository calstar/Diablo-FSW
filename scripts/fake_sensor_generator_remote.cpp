#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include <memory>
#include <signal.h>
#include <unistd.h>

#include "../external/shared/message_factory/MessageFactory.hpp"
#include "../comms/include/PTMessage.hpp"
#include "../comms/include/TCMessage.hpp"
#include "../comms/include/RTDMessage.hpp"
#include "../comms/include/IMUMessage.hpp"
#include "../comms/include/BarometerMessage.hpp"
#include "../comms/include/GPSMessage.hpp"
#include "../utl/Elodin.hpp"
#include "../utl/TCPSocket.hpp"
#include "../utl/dbConfig.hpp"

// Global variables for cleanup
std::unique_ptr<Socket> LocalSock;
std::unique_ptr<Socket> GroundStationSock;
bool running = true;

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    (void)signum; // Suppress unused parameter warning
    std::cout << "\nShutting down sensor generator..." << std::endl;
    running = false;
}

// Random number generator
std::random_device rd;
std::mt19937 gen(rd());

// PT Sensor Generator
void generatePTData() {
    std::uniform_real_distribution<double> pressure_dist(95000.0, 105000.0); // Pa
    std::uniform_real_distribution<double> temp_dist(20.0, 30.0); // C
    std::uniform_real_distribution<double> noise_dist(-0.1, 0.1);
    
    while (running) {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        double time_s = time_ns / 1e9;
        
        PTMessage pt_msg;
        double pressure = pressure_dist(gen) + noise_dist(gen);
        double temperature = temp_dist(gen) + noise_dist(gen);
        
        set_pt_measurement(pt_msg, time_s, pressure, temperature, time_ns);
        
        // Write to database
        std::array<uint8_t, 2> packet_id = {0x01, 0x00}; // PT sensor ID
        write_to_elodindb(packet_id, pt_msg);
        
        std::cout << "PT: P=" << pressure << " Pa, T=" << temperature << " C" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10 Hz
    }
}

// TC Sensor Generator
void generateTCData() {
    std::uniform_real_distribution<double> temp_dist(100.0, 200.0); // C
    std::uniform_real_distribution<double> voltage_dist(0.001, 0.010); // V
    std::uniform_real_distribution<double> noise_dist(-0.05, 0.05);
    
    while (running) {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        double time_s = time_ns / 1e9;
        
        TCMessage tc_msg;
        double temperature = temp_dist(gen) + noise_dist(gen);
        double voltage = voltage_dist(gen) + noise_dist(gen) * 0.001;
        uint8_t tc_type = 0; // Type K
        
        set_tc_measurement(tc_msg, time_s, temperature, voltage, tc_type, time_ns);
        
        // Write to database
        std::array<uint8_t, 2> packet_id = {0x02, 0x00}; // TC sensor ID
        write_to_elodindb(packet_id, tc_msg);
        
        std::cout << "TC: T=" << temperature << " C, V=" << voltage << " V" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 5 Hz
    }
}

// RTD Sensor Generator
void generateRTDData() {
    std::uniform_real_distribution<double> temp_dist(15.0, 35.0); // C
    std::uniform_real_distribution<double> resistance_dist(95.0, 105.0); // Ohm
    std::uniform_real_distribution<double> noise_dist(-0.02, 0.02);
    
    while (running) {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        double time_s = time_ns / 1e9;
        
        RTDMessage rtd_msg;
        double temperature = temp_dist(gen) + noise_dist(gen);
        double resistance = resistance_dist(gen) + noise_dist(gen);
        uint8_t rtd_type = 0; // PT100
        
        set_rtd_measurement(rtd_msg, time_s, temperature, resistance, rtd_type, time_ns);
        
        // Write to database
        std::array<uint8_t, 2> packet_id = {0x03, 0x00}; // RTD sensor ID
        write_to_elodindb(packet_id, rtd_msg);
        
        std::cout << "RTD: T=" << temperature << " C, R=" << resistance << " Ohm" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 2 Hz
    }
}

// IMU Sensor Generator
void generateIMUData() {
    std::uniform_real_distribution<double> accel_dist(-0.5, 0.5); // m/s^2
    std::uniform_real_distribution<double> gyro_dist(-0.1, 0.1); // rad/s
    std::uniform_real_distribution<double> noise_dist(-0.01, 0.01);
    
    while (running) {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        double time_s = time_ns / 1e9;
        
        IMUMessage imu_msg;
        std::array<double, 3> accel = {
            0.0 + accel_dist(gen) + noise_dist(gen),
            0.0 + accel_dist(gen) + noise_dist(gen),
            9.81 + accel_dist(gen) + noise_dist(gen) // Gravity
        };
        std::array<double, 3> gyro = {
            gyro_dist(gen) + noise_dist(gen),
            gyro_dist(gen) + noise_dist(gen),
            gyro_dist(gen) + noise_dist(gen)
        };
        
        set_imu_measurement(imu_msg, time_s, accel, gyro, time_ns);
        
        // Write to database
        std::array<uint8_t, 2> packet_id = {0x04, 0x00}; // IMU sensor ID
        write_to_elodindb(packet_id, imu_msg);
        
        std::cout << "IMU: Accel=[" << accel[0] << ", " << accel[1] << ", " << accel[2] 
                  << "], Gyro=[" << gyro[0] << ", " << gyro[1] << ", " << gyro[2] << "]" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100 Hz
    }
}

// Barometer Sensor Generator
void generateBarometerData() {
    std::uniform_real_distribution<double> pressure_dist(95000.0, 105000.0); // Pa
    std::uniform_real_distribution<double> altitude_dist(-10.0, 10.0); // m
    std::uniform_real_distribution<double> temp_dist(20.0, 30.0); // C
    std::uniform_real_distribution<double> noise_dist(-0.1, 0.1);
    
    while (running) {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        double time_s = time_ns / 1e9;
        
        BarometerMessage bar_msg;
        double pressure = pressure_dist(gen) + noise_dist(gen);
        double altitude = altitude_dist(gen) + noise_dist(gen);
        double temperature = temp_dist(gen) + noise_dist(gen);
        
        set_barometer_measurement(bar_msg, time_s, pressure, altitude, temperature, time_ns);
        
        // Write to database
        std::array<uint8_t, 2> packet_id = {0x05, 0x00}; // Barometer sensor ID
        write_to_elodindb(packet_id, bar_msg);
        
        std::cout << "Barometer: P=" << pressure << " Pa, Alt=" << altitude 
                  << " m, T=" << temperature << " C" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 20 Hz
    }
}

// GPS Sensor Generator
void generateGPSData() {
    std::uniform_real_distribution<double> lat_dist(37.7, 37.8); // San Francisco area
    std::uniform_real_distribution<double> lon_dist(-122.5, -122.4);
    std::uniform_real_distribution<double> alt_dist(0.0, 100.0); // m
    std::uniform_real_distribution<double> vel_dist(-5.0, 5.0); // m/s
    std::uniform_real_distribution<double> noise_dist(-0.001, 0.001);
    
    while (running) {
        auto now = std::chrono::high_resolution_clock::now();
        auto time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
        uint32_t time_gps = time_ns / 1000000; // Convert to ms
        
        // GPS Position
        GPSPositionMessage gps_pos_msg;
        double latitude = lat_dist(gen) + noise_dist(gen);
        double longitude = lon_dist(gen) + noise_dist(gen);
        double altitude = alt_dist(gen) + noise_dist(gen);
        
        set_gps_position_measurement(gps_pos_msg, time_gps, 1, latitude, longitude, 
                                   altitude, 2.5, 3.0, 8, time_ns);
        
        // Write to database
        std::array<uint8_t, 2> packet_id_pos = {0x06, 0x00}; // GPS Position ID
        write_to_elodindb(packet_id_pos, gps_pos_msg);
        
        // GPS Velocity
        GPSVelocityMessage gps_vel_msg;
        float velocity_x = vel_dist(gen) + noise_dist(gen);
        float velocity_y = vel_dist(gen) + noise_dist(gen);
        float velocity_z = vel_dist(gen) + noise_dist(gen);
        
        set_gps_velocity_measurement(gps_vel_msg, time_gps, velocity_x, velocity_y, 
                                   velocity_z, 0.5, time_ns);
        
        // Write to database
        std::array<uint8_t, 2> packet_id_vel = {0x07, 0x00}; // GPS Velocity ID
        write_to_elodindb(packet_id_vel, gps_vel_msg);
        
        std::cout << "GPS: Lat=" << latitude << ", Lon=" << longitude 
                  << ", Alt=" << altitude << ", Vel=[" << velocity_x << ", " 
                  << velocity_y << ", " << velocity_z << "]" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 1 Hz
    }
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string groundstation_ip = "127.0.0.1";
    int groundstation_port = 2240;
    
    if (argc >= 2) {
        groundstation_ip = argv[1];
    }
    if (argc >= 3) {
        groundstation_port = std::stoi(argv[2]);
    }
    
    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    std::cout << "🚀 Starting Remote Sensor Generator..." << std::endl;
    std::cout << "   Groundstation: " << groundstation_ip << ":" << groundstation_port << std::endl;
    
    // Initialize socket connection to remote Elodin database
    LocalSock = std::make_unique<Socket>(groundstation_ip.c_str(), groundstation_port);
    // Socket constructor automatically connects, no need for separate connect() call
    
    std::cout << "✅ Connected to groundstation database. Starting fake sensor generators..." << std::endl;
    
    // Generate database configuration (send vtable schemas)
    std::cout << "Generating database configuration..." << std::endl;
    cppGenerateDBConfig();
    std::cout << "Database configuration complete!" << std::endl;
    
    // Start sensor generator threads
    std::thread pt_thread(generatePTData);
    std::thread tc_thread(generateTCData);
    std::thread rtd_thread(generateRTDData);
    std::thread imu_thread(generateIMUData);
    std::thread barometer_thread(generateBarometerData);
    std::thread gps_thread(generateGPSData);
    
    // Wait for all threads to complete
    pt_thread.join();
    tc_thread.join();
    rtd_thread.join();
    imu_thread.join();
    barometer_thread.join();
    gps_thread.join();
    
    std::cout << "Sensor generator shutdown complete." << std::endl;
    return 0;
}
