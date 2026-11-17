#pragma once

#include <stddef.h>
#include <stdint.h>

namespace wristband {

// Order matches ROS sides: left, right, forward, back.
constexpr uint8_t kMotorPins[] = {0, 1, 20, 21};
constexpr size_t kMotorCount = sizeof(kMotorPins) / sizeof(kMotorPins[0]);
constexpr char kDirections[] = {'l', 'r', 'f', 'b'};
constexpr char kDeviceName[] = "naviGoatWristband";
constexpr char kServiceUuid[] = "0bbbe0d0-3a64-4f83-bc35-64314a26cb60";
constexpr char kWriteCharacteristicUuid[] = "0bbbe0d1-3a64-4f83-bc35-64314a26cb60";

constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr uint8_t kPwmResolutionBits = 8;
// Refresh each active side before this expires. Set to 0 to latch until stopped.
constexpr uint32_t kCommandTimeoutMs = 2000;

}  // namespace wristband
