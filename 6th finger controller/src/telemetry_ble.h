#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

#ifndef BLE_DEVICE_NAME
#define BLE_DEVICE_NAME "ESP32-Flex"
#endif

#ifndef BLE_LED_PIN
#define BLE_LED_PIN 2
#endif

struct TelemetryValues
{
    float flex_avg_ohm = 0.0f;
    float servo_deg = 0.0f;
};

class TelemetryBLE
{
public:
    static constexpr const char *SERVICE_UUID = "6f1a0001-7e03-4a5a-9c5a-8b1f9c1a0001";
    static constexpr const char *FLEX_UUID = "6f1a0002-7e03-4a5a-9c5a-8b1f9c1a0002";
    static constexpr const char *SERVO_UUID = "6f1a0003-7e03-4a5a-9c5a-8b1f9c1a0003";

    void begin(const char *deviceName = BLE_DEVICE_NAME);
    void setValues(const TelemetryValues &v);
    void loop();

    bool isClientConnected() const { return clientConnected; }
    bool hasSubscribers() const;

private:
    NimBLEServer *server = nullptr;
    NimBLEService *service = nullptr;
    NimBLECharacteristic *chFlex = nullptr;
    NimBLECharacteristic *chServo = nullptr;

    TelemetryValues lastSent{};
    TelemetryValues pending{};
    bool havePending = false;

    uint32_t lastNotifyMs = 0;
    uint32_t notifyIntervalMs = 50;

    bool clientConnected = false;

    enum class LedMode
    {
        Advertising,
        Connected,
        Notifying
    };
    LedMode ledMode = LedMode::Advertising;
    uint32_t lastBlink = 0;
    void updateLed();
    void setLed(bool on);
    void createGatt(const char *deviceName);
    static void floatToLE(float f, uint8_t out[4]);

public:
    void onConnect();
    void onDisconnect();
};
