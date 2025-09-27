#ifndef PT_MESSAGE_HPP
#define PT_MESSAGE_HPP

#include <array>
#include <cstdint>

#include "../../external/shared/message_factory/MessageFactory.hpp"

/**
 * @brief PT Location enumeration for engine system
 */
enum class PTLocation {
    PRESSURANT_TANK = 0,    // Pressurant tank PT
    KERO_INLET = 1,         // Kero Inlet PT
    KERO_OUTLET = 2,        // Kero Outlet PT
    LOX_INLET = 3,          // Lox Inlet PT
    LOX_OUTLET = 4,         // Lox Outlet PT
    INJECTOR = 5,           // Injector PT
    CHAMBER_WALL_1 = 6,     // Chamber Wall PT #1
    CHAMBER_WALL_2 = 7,     // Chamber Wall PT #2
    NOZZLE_EXIT = 8,        // Nozzle Exit PT
    UNKNOWN = 9             // Unknown/Unused
};

/**
 * @brief Pressure Transducer Message - Raw voltage system
 *
 * Contains raw voltage readings from PT sensors (calibration applied later)
 */
using PTMessage =
    MessageFactory<uint64_t,   // (0) timestamp_ns - monotonic timestamp in nanoseconds
                   uint8_t,    // (1) sensor_id - PT sensor identifier (0-8 for 9 PTs)
                   double,     // (2) raw_voltage_v - raw voltage reading in Volts
                   uint8_t>;   // (3) pt_location - PT location enum value

// Function to set PT sensor measurements with raw voltage
static void set_pt_measurement(PTMessage& message, uint64_t timestamp_ns, uint8_t sensor_id, 
                               double raw_voltage_v, PTLocation location) {
    message.setField<0>(timestamp_ns);
    message.setField<1>(sensor_id);
    message.setField<2>(raw_voltage_v);
    message.setField<3>(static_cast<uint8_t>(location));
}

// Helper function to get PT location name
static std::string getPTLocationName(PTLocation location) {
    switch (location) {
        case PTLocation::PRESSURANT_TANK: return "Pressurant Tank";
        case PTLocation::KERO_INLET: return "Kero Inlet";
        case PTLocation::KERO_OUTLET: return "Kero Outlet";
        case PTLocation::LOX_INLET: return "Lox Inlet";
        case PTLocation::LOX_OUTLET: return "Lox Outlet";
        case PTLocation::INJECTOR: return "Injector";
        case PTLocation::CHAMBER_WALL_1: return "Chamber Wall #1";
        case PTLocation::CHAMBER_WALL_2: return "Chamber Wall #2";
        case PTLocation::NOZZLE_EXIT: return "Nozzle Exit";
        case PTLocation::UNKNOWN: return "Unknown";
        default: return "Invalid";
    }
}

static PTMessage generateTestMessagePT() {
    PTMessage message;
    set_pt_measurement(message, 0, 0, 2.5, PTLocation::PRESSURANT_TANK); // 2.5V raw reading
    return message;
}

// Specialized PT messages for different locations
using PTChamberMessage = PTMessage;       // Chamber pressure PT
using PTFuelInletMessage = PTMessage;     // Fuel inlet PT
using PTOxInletMessage = PTMessage;       // Oxidizer inlet PT
using PTCoolantInletMessage = PTMessage;  // Coolant inlet PT
using PTIgniterMessage = PTMessage;       // Igniter PT

#endif  // PT_MESSAGE_HPP