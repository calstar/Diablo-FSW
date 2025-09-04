#ifndef PT_MESSAGE_HPP
#define PT_MESSAGE_HPP

#include <array>
#include <cstdint>
#include "../../external/shared/message_factory/MessageFactory.hpp"

/**
 * @brief Pressure/Temperature sensor message
 * PT sensors measure both pressure and temperature
 */
using PTMessage = MessageFactory<
    double,     // (0) time_pt (s) - timestamp
    double,     // (1) pressure (Pa) - pressure reading
    double,     // (2) temperature (C) - temperature reading
    uint64_t>;  // (3) time_monotonic (ns) - monotonic timestamp

// Function to set PT sensor measurements
static void set_pt_measurement(
    PTMessage& message, 
    double time_pt,
    double pressure,
    double temperature,
    uint64_t time_monotonic) {
    message.setField<0>(time_pt);
    message.setField<1>(pressure);
    message.setField<2>(temperature);
    message.setField<3>(time_monotonic);
}

static PTMessage generateTestMessagePT() {
    PTMessage message;
    set_pt_measurement(message, 0.0, 101325.0, 25.0, 0);
    return message;
}

#endif  // PT_MESSAGE_HPP
