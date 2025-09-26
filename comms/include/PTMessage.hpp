#ifndef PT_MESSAGE_HPP
#define PT_MESSAGE_HPP

#include <array>
#include <cstdint>
#include "../../external/shared/message_factory/MessageFactory.hpp"

/**
 * @brief Pressure Transducer Message
 * 
 * Contains pressure measurements from various PT sensors with calibration data
 */
using PTMessage = MessageFactory<
    double,     // (0) timestamp (s) - timestamp
    uint8_t,    // (1) sensor_id - PT sensor identifier
    double,     // (2) raw_voltage (V) - raw voltage reading
    double,     // (3) pressure (Pa) - calibrated pressure
    double,     // (4) pressure_uncertainty (Pa) - measurement uncertainty
    double,     // (5) temperature (°C) - sensor temperature
    double,     // (6) calibration_quality (0-1) - calibration quality
    bool,       // (7) calibration_valid - calibration validity
    double,     // (8) drift_detected - drift detection flag
    uint8_t,    // (9) sensor_health - sensor health status
    double,     // (10) environmental_factor - environmental correction factor
    uint64_t>;  // (11) time_monotonic (ns) - monotonic timestamp

// Function to set PT sensor measurements
static void set_pt_measurement(
    PTMessage& message,
    double timestamp,
    uint8_t sensor_id,
    double raw_voltage,
    double pressure,
    double pressure_uncertainty,
    double temperature,
    double calibration_quality,
    bool calibration_valid,
    double drift_detected,
    uint8_t sensor_health,
    double environmental_factor,
    uint64_t time_monotonic) {
    message.setField<0>(timestamp);
    message.setField<1>(sensor_id);
    message.setField<2>(raw_voltage);
    message.setField<3>(pressure);
    message.setField<4>(pressure_uncertainty);
    message.setField<5>(temperature);
    message.setField<6>(calibration_quality);
    message.setField<7>(calibration_valid);
    message.setField<8>(drift_detected);
    message.setField<9>(sensor_health);
    message.setField<10>(environmental_factor);
    message.setField<11>(time_monotonic);
}

static PTMessage generateTestMessagePT() {
    PTMessage message;
    set_pt_measurement(message, 0.0, 1, 2.5, 101325.0, 100.0, 25.0, 0.95, 
                      true, 0.0, 0, 1.0, 0);
    return message;
}

// Specialized PT messages for different locations
using PTChamberMessage = PTMessage;  // Chamber pressure PT
using PTFuelInletMessage = PTMessage; // Fuel inlet PT
using PTOxInletMessage = PTMessage;   // Oxidizer inlet PT
using PTCoolantInletMessage = PTMessage; // Coolant inlet PT
using PTIgniterMessage = PTMessage;   // Igniter PT

#endif // PT_MESSAGE_HPP