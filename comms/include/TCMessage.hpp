#ifndef TC_MESSAGE_HPP
#define TC_MESSAGE_HPP

#include <array>
#include <cstdint>
#include "../../external/shared/message_factory/MessageFactory.hpp"

/**
 * @brief Thermocouple sensor message
 * TC sensors measure temperature using thermocouple principles
 */
using TCMessage = MessageFactory<
    double,     // (0) time_tc (s) - timestamp
    double,     // (1) temperature (C) - temperature reading
    double,     // (2) voltage (V) - raw voltage reading
    uint8_t,    // (3) tc_type - thermocouple type (K, J, T, etc.)
    uint64_t>;  // (4) time_monotonic (ns) - monotonic timestamp

// Function to set TC sensor measurements
static void set_tc_measurement(
    TCMessage& message, 
    double time_tc,
    double temperature,
    double voltage,
    uint8_t tc_type,
    uint64_t time_monotonic) {
    message.setField<0>(time_tc);
    message.setField<1>(temperature);
    message.setField<2>(voltage);
    message.setField<3>(tc_type);
    message.setField<4>(time_monotonic);
}

static TCMessage generateTestMessageTC() {
    TCMessage message;
    set_tc_measurement(message, 0.0, 150.0, 0.006, 0, 0); // Type K thermocouple
    return message;
}

#endif  // TC_MESSAGE_HPP
