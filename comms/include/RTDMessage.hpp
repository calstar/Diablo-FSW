#ifndef RTD_MESSAGE_HPP
#define RTD_MESSAGE_HPP

#include <array>
#include <cstdint>
#include "../../external/shared/message_factory/MessageFactory.hpp"

/**
 * @brief RTD (Resistance Temperature Detector) sensor message
 * RTD sensors measure temperature using resistance changes
 */
using RTDMessage = MessageFactory<
    double,     // (0) time_rtd (s) - timestamp
    double,     // (1) temperature (C) - temperature reading
    double,     // (2) resistance (Ohm) - resistance reading
    uint8_t,    // (3) rtd_type - RTD type (PT100, PT1000, etc.)
    uint64_t>;  // (4) time_monotonic (ns) - monotonic timestamp

// Function to set RTD sensor measurements
static void set_rtd_measurement(
    RTDMessage& message, 
    double time_rtd,
    double temperature,
    double resistance,
    uint8_t rtd_type,
    uint64_t time_monotonic) {
    message.setField<0>(time_rtd);
    message.setField<1>(temperature);
    message.setField<2>(resistance);
    message.setField<3>(rtd_type);
    message.setField<4>(time_monotonic);
}

static RTDMessage generateTestMessageRTD() {
    RTDMessage message;
    set_rtd_measurement(message, 0.0, 25.0, 100.0, 0, 0); // PT100 at 25C
    return message;
}

#endif  // RTD_MESSAGE_HPP
