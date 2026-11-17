#include <Arduino.h>

#include "wristband_config.h"

using namespace wristband;

#if defined(WRISTBAND_MOTOR_TEST) && WRISTBAND_MOTOR_TEST

// Standalone wiring check, matching the original motor test. No BLE required.
void setup() {
    for (int pin : kMotorPins) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW);
    }
}

void loop() {
    for (int pin : kMotorPins) {
        digitalWrite(pin, HIGH);
        delay(1000);
        digitalWrite(pin, LOW);
        delay(500);
    }
    delay(2000);
}

#else

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "vibration_command.h"

namespace {

struct MotorState {
    uint8_t magnitude;
    uint32_t updatedAt;
};

// BLE callbacks run in a different task. Only loop() touches PWM hardware.
portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;
MotorState motors[kMotorCount];
bool connected = false;
bool restartAdvertising = false;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer*) override {
        portENTER_CRITICAL(&stateMux);
        for (auto& motor : motors) {
            motor = {};
        }
        connected = true;
        portEXIT_CRITICAL(&stateMux);
    }

    void onDisconnect(BLEServer*) override {
        portENTER_CRITICAL(&stateMux);
        connected = false;
        for (auto& motor : motors) {
            motor = {};
        }
        restartAdvertising = true;
        portEXIT_CRITICAL(&stateMux);
    }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* characteristic) override {
        const std::string value = characteristic->getValue();
        VibrationCommand command;
        if (!parseCommand(value.data(), value.size(), command)) {
            return;
        }

        const uint32_t now = millis();
        portENTER_CRITICAL(&stateMux);
        if (connected) {
            motors[command.side] = {command.magnitude, now};
        }
        portEXIT_CRITICAL(&stateMux);
    }
};

ServerCallbacks serverCallbacks;
CommandCallbacks commandCallbacks;

}  // namespace

void setup() {
    for (size_t i = 0; i < kMotorCount; ++i) {
        pinMode(kMotorPins[i], OUTPUT);
        digitalWrite(kMotorPins[i], LOW);
        ledcSetup(i, kPwmFrequencyHz, kPwmResolutionBits);
        ledcWrite(i, 0);
        ledcAttachPin(kMotorPins[i], i);
    }

    // Keep UART0 free: its RX/TX pins (20/21) drive two of the motors.
    BLEDevice::init(kDeviceName);
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(&serverCallbacks);
    BLEService* service = server->createService(kServiceUuid);
    BLECharacteristic* characteristic = service->createCharacteristic(
        kWriteCharacteristicUuid,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    characteristic->setCallbacks(&commandCallbacks);
    service->start();

    // A 128-bit service UUID and the full name do not fit in one 31-byte packet.
    BLEAdvertisementData advertisement;
    advertisement.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
    advertisement.setCompleteServices(BLEUUID(kServiceUuid));
    BLEAdvertisementData scanResponse;
    scanResponse.setName(kDeviceName);
    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->setAdvertisementData(advertisement);
    advertising->setScanResponseData(scanResponse);
    advertising->setScanResponse(true);
    advertising->start();
}

void loop() {
    uint8_t duties[kMotorCount];
    bool advertise;
    portENTER_CRITICAL(&stateMux);
    const uint32_t now = millis();
    for (size_t i = 0; i < kMotorCount; ++i) {
        if (!connected || (kCommandTimeoutMs != 0 &&
                           static_cast<uint32_t>(now - motors[i].updatedAt) >= kCommandTimeoutMs)) {
            motors[i].magnitude = 0;
        }
        duties[i] = motors[i].magnitude;
    }
    advertise = restartAdvertising;
    restartAdvertising = false;
    portEXIT_CRITICAL(&stateMux);

    for (size_t i = 0; i < kMotorCount; ++i) {
        ledcWrite(i, duties[i]);
    }
    if (advertise) {
        BLEDevice::startAdvertising();
    }
    delay(5);
}

#endif
