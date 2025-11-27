#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <Preferences.h>

#include "settings.h"
#include "control.h"

#define BLE_LED_PIN 2

class CfgInCB;
class ConnCB;

class BleApp
{
public:
    BleApp() = default;

    void begin(const char *name);
    void loop();

    void sendTelemetry(const ControlTelemetry &t);

    const Settings &getSettings() const { return current; }

    std::function<void(const Settings &)> onSettingsChanged;

private:
    friend class CfgInCB;
    friend class ConnCB;

    NimBLEServer *server = nullptr;

    NimBLECharacteristic *chCfgIn = nullptr;
    NimBLECharacteristic *chCfgOut = nullptr;
    NimBLECharacteristic *chAck = nullptr;
    NimBLECharacteristic *chTele = nullptr;

    Settings current{};
    Preferences nvs;

    bool receivingChunks = false;
    String chunkBuffer;

    StaticJsonDocument<512> teleJson;
    bool teleDirty = false;
    uint32_t lastTeleSend = 0;

    enum class LedMode
    {
        Adv,
        Conn,
        Notify
    };
    LedMode led = LedMode::Adv;
    uint32_t lastBlink = 0;

private:
    void loadSettings();
    void saveSettings();
    void setupBLE();
    void startAdvertising();

    void handleChunk(const std::string &s);
    void sendJsonChunked(const JsonDocument &doc, NimBLECharacteristic *ch);

    void applyIncomingJson(const JsonDocument &doc);

    void sendAck(bool ok);

    void makeTelemetryJson(const ControlTelemetry &t);

    void updateLed();
};
