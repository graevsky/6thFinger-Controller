#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <functional>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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

    StaticJsonDocument<2048> teleJson;
    bool teleDirty = false;
    uint32_t lastTeleSend = 0;

    SemaphoreHandle_t txMutex = nullptr;

    volatile uint32_t telePauseUntilMs = 0;
    static constexpr uint32_t TELE_MIN_PERIOD_MS = 100;

    bool teleEnabled = false;

    volatile bool pendingSendConfig = false;

    volatile bool pendingSendAck = false;
    volatile bool pendingAckOk = true;
    volatile uint32_t pendingAckAtMs = 0;

    static constexpr uint32_t ACK_DELAY_MS = 250;
    static constexpr uint32_t PAUSE_TELE_MS = 6000;

    bool authed = false;
    bool isAuthed() const { return (current.pinCode == 0) || authed; }

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

    void sendJsonChunked(
        const JsonDocument &doc,
        NimBLECharacteristic *ch,
        bool pauseTele,
        bool useIndicate);

    void applyIncomingJson(const JsonDocument &doc);
    void sendConfig();
    void sendAck(bool ok);

    void sendAuthAck(bool ok);

    void makeTelemetryJson(const ControlTelemetry &t);

    void updateLed();

    static void callNotify(NimBLECharacteristic *ch);
    static void callIndicate(NimBLECharacteristic *ch);
};
