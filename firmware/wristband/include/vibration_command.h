#pragma once

#include "wristband_config.h"

namespace wristband {

struct VibrationCommand {
    size_t side;
    uint8_t magnitude;
};

// The ROS bridge sends exactly one complete ASCII frame per GATT write.
inline bool parseCommand(const char* data, size_t length, VibrationCommand& command) {
    if (length < 5 || length > 7 || data[0] != '<' || data[2] != ':' ||
        data[length - 1] != '>') {
        return false;
    }

    size_t side = 0;
    while (side < kMotorCount && data[1] != kDirections[side]) {
        ++side;
    }
    if (side == kMotorCount) {
        return false;
    }

    unsigned magnitude = 0;
    for (size_t i = 3; i < length - 1; ++i) {
        if (data[i] < '0' || data[i] > '9') {
            return false;
        }
        magnitude = magnitude * 10 + static_cast<unsigned>(data[i] - '0');
    }
    if (magnitude > 255) {
        return false;
    }

    command = {side, static_cast<uint8_t>(magnitude)};
    return true;
}

}  // namespace wristband
