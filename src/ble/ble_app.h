// BLE application layer.
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <functional>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "../config/settings.h"
#include "../config/telemetry.h"

// Status LED used for advertising, connection, and notify feedback.
#define BLE_LED_PIN 2

class CfgInCB;
class ConnCB;
class ServoLiveCB;

class BleApp
{
public:
    BleApp() = default;

    // JSON buffer sizes sized for the 5-pair configuration model.
    static constexpr size_t CFG_JSON_CAPACITY = 8192;
    static constexpr size_t TELE_JSON_CAPACITY = 8192;
    static constexpr size_t ACK_JSON_CAPACITY = 128;

    // Initialize BLE, settings storage, and GATT services.
    void begin(const char *name);
    // Service deferred BLE work from the main loop.
    void loop();

    // Queue a telemetry snapshot for transmission.
    void sendTelemetry(const ControlTelemetry &t);

    // Expose the currently loaded settings snapshot.
    const Settings &getSettings() const { return current; }

    // Callbacks back into the application when BLE changes state.
    std::function<void(const Settings &)> onSettingsChanged;
    std::function<void(int idx, int deg, bool stop)> onServoLive;

private:
    friend class CfgInCB;
    friend class ConnCB;
    friend class ServoLiveCB;

    // Core GATT objects for config, ACK, telemetry, and live servo control.
    NimBLEServer *server = nullptr;

    NimBLECharacteristic *chCfgIn = nullptr;
    NimBLECharacteristic *chCfgOut = nullptr;
    NimBLECharacteristic *chAck = nullptr;
    NimBLECharacteristic *chTele = nullptr;
    NimBLECharacteristic *chServoLive = nullptr;

    // Persisted configuration snapshot and its NVS handle.
    Settings current{};
    Preferences nvs;

    // Chunked JSON receive state.
    bool receivingChunks = false;
    String chunkBuffer;

    // Telemetry snapshot buffer and send throttling state.
    StaticJsonDocument<TELE_JSON_CAPACITY> teleJson;
    bool teleDirty = false;
    uint32_t lastTeleSend = 0;

    // Serialize BLE sends so config and telemetry do not overlap.
    SemaphoreHandle_t txMutex = nullptr;

    // Telemetry pause window used while chunked config traffic is active.
    volatile uint32_t telePauseUntilMs = 0;
    static constexpr uint32_t TELE_MIN_PERIOD_MS = 100;

    // Runtime flag controlled by the mobile app.
    bool teleEnabled = false;

    // Deferred config/ACK send state handled from loop().
    volatile bool pendingSendConfig = false;

    volatile bool pendingSendAck = false;
    volatile bool pendingAckOk = true;
    volatile uint32_t pendingAckAtMs = 0;

    char pendingAckFor[16] = {0};
    volatile int pendingAckTeleVal = -1;

    // Timing constants for ACK ordering and telemetry suppression.
    static constexpr uint32_t ACK_DELAY_MS = 250;
    static constexpr uint32_t PAUSE_TELE_MS = 6000;

    // Simple session auth state; pinCode==0 means auth is disabled.
    bool authed = false;
    // True when BLE commands are allowed for the current session.
    bool isAuthed() const { return (current.pinCode == 0) || authed; }

    enum class LedMode
    {
        Adv,
        Conn,
        Notify
    };

    // LED mode state machine for the built-in indicator.
    LedMode led = LedMode::Adv;
    uint32_t lastBlink = 0;

private:
    // Settings, BLE setup, and receive handlers.
    void loadSettings();
    void saveSettings();
    void setupBLE();
    void startAdvertising();

    void handleChunk(const std::string &s);
    void handleServoLive(const std::string &s);

    // Chunked JSON .
    void sendJsonChunked(
        const JsonDocument &doc,
        NimBLECharacteristic *ch,
        bool pauseTele,
        bool useIndicate);

    // Helpers for applying config changes and emitting responses.
    void applyIncomingJson(const JsonDocument &doc);
    void sendConfig();

    void sendAck(bool ok, const char *forTag = nullptr, int teleEnabledVal = -1);
    void sendAuthAck(bool ok);

    // Build the flattened telemetry JSON object from the internal snapshot.
    void makeTelemetryJson(const ControlTelemetry &t);

    // Small LED and BLE helper wrappers.
    void updateLed();

    static void callNotify(NimBLECharacteristic *ch);
    static void callIndicate(NimBLECharacteristic *ch);

private:
    // Capture ACK metadata so the delayed loop() sender can recreate it later.
    inline void setPendingAckMeta(const char *forTag, int teleVal = -1)
    {
        if (forTag)
        {
            strncpy(pendingAckFor, forTag, sizeof(pendingAckFor) - 1);
            pendingAckFor[sizeof(pendingAckFor) - 1] = 0;
        }
        else
        {
            pendingAckFor[0] = 0;
        }
        pendingAckTeleVal = teleVal;
    }
};
