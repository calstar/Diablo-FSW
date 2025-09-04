#pragma once
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "Elodin.hpp"
#include "TCPSocket.hpp"
#include "db.hpp"

using namespace vtable;
using namespace vtable::builder;

extern std::unique_ptr<Socket> LocalSock;
extern std::unique_ptr<Socket> GroundStationSock;

static std::unordered_map<std::string, std::string> structToEntityID;
static std::unordered_set<std::string> addedComponents;

/*
 * @brief a template fn to send messages to the DB using
 * Elodin defined primitives (Msg, .encode_vec())
 */
template <typename T>
void send(T msg) {
    auto buf = Msg(msg).encode_vec();
    // Send to local database
    LocalSock->write_all(buf.data(), buf.size(), "dbConfig.hpp db setup");
}

void cppGenerateDBConfig() {
    std::cout << "Generating database configuration..." << std::endl;
    
    // ══════════════════════════════════════════════════════════════════════════════
    // IMUMessage
    // Fields: time_imu, accelerometer (x, y, z), gyroscope (x, y, z), time_monotonic
    // ══════════════════════════════════════════════════════════════════════════════
    
    auto imuTable = builder::vtable({
        raw_field(0, 8, schema(PrimType::F64(), {}, pair(0x01, "time_imu"))),
        raw_field(8, 8, schema(PrimType::F64(), {}, pair(0x01, "accelerometer_x"))),
        raw_field(16, 8, schema(PrimType::F64(), {}, pair(0x01, "accelerometer_y"))),
        raw_field(24, 8, schema(PrimType::F64(), {}, pair(0x01, "accelerometer_z"))),
        raw_field(32, 8, schema(PrimType::F64(), {}, pair(0x01, "gyroscope_x"))),
        raw_field(40, 8, schema(PrimType::F64(), {}, pair(0x01, "gyroscope_y"))),
        raw_field(48, 8, schema(PrimType::F64(), {}, pair(0x01, "gyroscope_z"))),
        raw_field(56, 8, schema(PrimType::U64(), {}, pair(0x01, "time_monotonic"))),
    });
    
    send(VTableMsg{
        .id = {0x01, 0x00},
        .vtable = imuTable,
    });
    
    send(set_component_name("time_imu"));
    send(set_component_name("accelerometer_x"));
    send(set_component_name("accelerometer_y"));
    send(set_component_name("accelerometer_z"));
    send(set_component_name("gyroscope_x"));
    send(set_component_name("gyroscope_y"));
    send(set_component_name("gyroscope_z"));
    send(set_component_name("time_monotonic"));
    send(set_entity_name(0x01, "IMU"));
    
    // ══════════════════════════════════════════════════════════════════════════════
    // PTMessage (Pressure/Temperature)
    // Fields: time_pt, pressure, temperature, time_monotonic
    // ══════════════════════════════════════════════════════════════════════════════
    
    auto ptTable = builder::vtable({
        raw_field(0, 8, schema(PrimType::F64(), {}, pair(0x02, "time_pt"))),
        raw_field(8, 8, schema(PrimType::F64(), {}, pair(0x02, "pressure"))),
        raw_field(16, 8, schema(PrimType::F64(), {}, pair(0x02, "temperature"))),
        raw_field(24, 8, schema(PrimType::U64(), {}, pair(0x02, "time_monotonic"))),
    });
    
    send(VTableMsg{
        .id = {0x02, 0x00},
        .vtable = ptTable,
    });
    
    send(set_component_name("time_pt"));
    send(set_component_name("pressure"));
    send(set_component_name("temperature"));
    send(set_component_name("time_monotonic"));
    send(set_entity_name(0x02, "PT"));
    
    // ══════════════════════════════════════════════════════════════════════════════
    // TCMessage (Thermocouple)
    // Fields: time_tc, temperature, voltage, time_monotonic
    // ══════════════════════════════════════════════════════════════════════════════
    
    auto tcTable = builder::vtable({
        raw_field(0, 8, schema(PrimType::F64(), {}, pair(0x03, "time_tc"))),
        raw_field(8, 8, schema(PrimType::F64(), {}, pair(0x03, "temperature"))),
        raw_field(16, 8, schema(PrimType::F64(), {}, pair(0x03, "voltage"))),
        raw_field(24, 8, schema(PrimType::U64(), {}, pair(0x03, "time_monotonic"))),
    });
    
    send(VTableMsg{
        .id = {0x03, 0x00},
        .vtable = tcTable,
    });
    
    send(set_component_name("time_tc"));
    send(set_component_name("temperature"));
    send(set_component_name("voltage"));
    send(set_component_name("time_monotonic"));
    send(set_entity_name(0x03, "TC"));
    
    // ══════════════════════════════════════════════════════════════════════════════
    // RTDMessage (Resistance Temperature Detector)
    // Fields: time_rtd, temperature, resistance, time_monotonic
    // ══════════════════════════════════════════════════════════════════════════════
    
    auto rtdTable = builder::vtable({
        raw_field(0, 8, schema(PrimType::F64(), {}, pair(0x04, "time_rtd"))),
        raw_field(8, 8, schema(PrimType::F64(), {}, pair(0x04, "temperature"))),
        raw_field(16, 8, schema(PrimType::F64(), {}, pair(0x04, "resistance"))),
        raw_field(24, 8, schema(PrimType::U64(), {}, pair(0x04, "time_monotonic"))),
    });
    
    send(VTableMsg{
        .id = {0x04, 0x00},
        .vtable = rtdTable,
    });
    
    send(set_component_name("time_rtd"));
    send(set_component_name("temperature"));
    send(set_component_name("resistance"));
    send(set_component_name("time_monotonic"));
    send(set_entity_name(0x04, "RTD"));
    
    // ══════════════════════════════════════════════════════════════════════════════
    // BarometerMessage
    // Fields: time_bar, pressure, altitude, temperature, time_monotonic
    // ══════════════════════════════════════════════════════════════════════════════
    
    auto barTable = builder::vtable({
        raw_field(0, 8, schema(PrimType::F64(), {}, pair(0x05, "time_bar"))),
        raw_field(8, 8, schema(PrimType::F64(), {}, pair(0x05, "pressure"))),
        raw_field(16, 8, schema(PrimType::F64(), {}, pair(0x05, "altitude"))),
        raw_field(24, 8, schema(PrimType::F64(), {}, pair(0x05, "temperature"))),
        raw_field(32, 8, schema(PrimType::U64(), {}, pair(0x05, "time_monotonic"))),
    });
    
    send(VTableMsg{
        .id = {0x05, 0x00},
        .vtable = barTable,
    });
    
    send(set_component_name("time_bar"));
    send(set_component_name("pressure"));
    send(set_component_name("altitude"));
    send(set_component_name("temperature"));
    send(set_component_name("time_monotonic"));
    send(set_entity_name(0x05, "Barometer"));
    
    // ══════════════════════════════════════════════════════════════════════════════
    // GPSPositionMessage
    // Fields: time_gps_pos, latitude, longitude, altitude, time_monotonic
    // ══════════════════════════════════════════════════════════════════════════════
    
    auto gpsPosTable = builder::vtable({
        raw_field(0, 8, schema(PrimType::F64(), {}, pair(0x06, "time_gps_pos"))),
        raw_field(8, 8, schema(PrimType::F64(), {}, pair(0x06, "latitude"))),
        raw_field(16, 8, schema(PrimType::F64(), {}, pair(0x06, "longitude"))),
        raw_field(24, 8, schema(PrimType::F64(), {}, pair(0x06, "altitude"))),
        raw_field(32, 8, schema(PrimType::U64(), {}, pair(0x06, "time_monotonic"))),
    });
    
    send(VTableMsg{
        .id = {0x06, 0x00},
        .vtable = gpsPosTable,
    });
    
    send(set_component_name("time_gps_pos"));
    send(set_component_name("latitude"));
    send(set_component_name("longitude"));
    send(set_component_name("altitude"));
    send(set_component_name("time_monotonic"));
    send(set_entity_name(0x06, "GPS_Position"));
    
    // ══════════════════════════════════════════════════════════════════════════════
    // GPSVelocityMessage
    // Fields: time_gps_vel, velocity_x, velocity_y, velocity_z, time_monotonic
    // ══════════════════════════════════════════════════════════════════════════════
    
    auto gpsVelTable = builder::vtable({
        raw_field(0, 8, schema(PrimType::F64(), {}, pair(0x07, "time_gps_vel"))),
        raw_field(8, 8, schema(PrimType::F64(), {}, pair(0x07, "velocity_x"))),
        raw_field(16, 8, schema(PrimType::F64(), {}, pair(0x07, "velocity_y"))),
        raw_field(24, 8, schema(PrimType::F64(), {}, pair(0x07, "velocity_z"))),
        raw_field(32, 8, schema(PrimType::U64(), {}, pair(0x07, "time_monotonic"))),
    });
    
    send(VTableMsg{
        .id = {0x07, 0x00},
        .vtable = gpsVelTable,
    });
    
    send(set_component_name("time_gps_vel"));
    send(set_component_name("velocity_x"));
    send(set_component_name("velocity_y"));
    send(set_component_name("velocity_z"));
    send(set_component_name("time_monotonic"));
    send(set_entity_name(0x07, "GPS_Velocity"));
    
    std::cout << "Database configuration complete!" << std::endl;
}