#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <functional>

static constexpr const char *UUID_TELEMETRY_SVC = "6f1a0001-7e03-4a5a-9c5a-8b1f9c1a0001";
static constexpr const char *UUID_FLEX = "6f1a0002-7e03-4a5a-9c5a-8b1f9c1a0002";
static constexpr const char *UUID_SERVO = "6f1a0003-7e03-4a5a-9c5a-8b1f9c1a0003";
static constexpr const char *UUID_FSR = "6f1a0004-7e03-4a5a-9c5a-8b1f9c1a0004";

static constexpr const char *UUID_CFG_SVC = "6f1a1000-7e03-4a5a-9c5a-8b1f9c1a1000";
static constexpr const char *UUID_CFG_FSR_PIN = "6f1a1001-7e03-4a5a-9c5a-8b1f9c1a1001";
static constexpr const char *UUID_CFG_FSR_PULL = "6f1a1002-7e03-4a5a-9c5a-8b1f9c1a1002";
static constexpr const char *UUID_CFG_FSR_START = "6f1a1003-7e03-4a5a-9c5a-8b1f9c1a1003";
static constexpr const char *UUID_CFG_FSR_MAX = "6f1a1004-7e03-4a5a-9c5a-8b1f9c1a1004";

static constexpr const char *UUID_CFG_FLEX_PIN = "6f1a1011-7e03-4a5a-9c5a-8b1f9c1a1011";
static constexpr const char *UUID_CFG_FLEX_FLAT = "6f1a1012-7e03-4a5a-9c5a-8b1f9c1a1012";
static constexpr const char *UUID_CFG_FLEX_BEND = "6f1a1013-7e03-4a5a-9c5a-8b1f9c1a1013";
static constexpr const char *UUID_CFG_VIB_PIN = "6f1a1021-7e03-4a5a-9c5a-8b1f9c1a1021";
static constexpr const char *UUID_CFG_VIB_FREQ = "6f1a1022-7e03-4a5a-9c5a-8b1f9c1a1022";
static constexpr const char *UUID_CFG_VIB_THRESH = "6f1a1023-7e03-4a5a-9c5a-8b1f9c1a1023";
static constexpr const char *UUID_CFG_VIB_PWR = "6f1a1024-7e03-4a5a-9c5a-8b1f9c1a1024";
static constexpr const char *UUID_CFG_SERVO_PIN = "6f1a1031-7e03-4a5a-9c5a-8b1f9c1a1031";
static constexpr const char *UUID_CFG_SERVO_MIN = "6f1a1032-7e03-4a5a-9c5a-8b1f9c1a1032";
static constexpr const char *UUID_CFG_SERVO_MAX = "6f1a1033-7e03-4a5a-9c5a-8b1f9c1a1033";
static constexpr const char *UUID_CFG_SERVO_MAN = "6f1a1034-7e03-4a5a-9c5a-8b1f9c1a1034";
static constexpr const char *UUID_CFG_SERVO_MAN_D = "6f1a1035-7e03-4a5a-9c5a-8b1f9c1a1035";
static constexpr const char *UUID_CFG_APPLY = "6f1a10ff-7e03-4a5a-9c5a-8b1f9c1a10ff";

#ifndef BLE_DEVICE_NAME
#define BLE_DEVICE_NAME "ESP32-Flex6"
#endif
#ifndef BLE_LED_PIN
#define BLE_LED_PIN 2
#endif

struct Settings
{
    uint8_t fsrPin = 34;
    uint16_t fsrPullupOhm = 4700;
    uint32_t fsrStartOhm = 100000;
    uint32_t fsrMaxOhm = 20000;

    uint8_t flexPin = 35;
    uint32_t flexFlatOhm = 45000;
    uint32_t flexBendOhm = 33400;

    uint8_t vibroPin = 25;
    uint16_t vibroFreqHz = 10;
    uint16_t vibroThreshold = 50;
    uint8_t vibroPowerPct = 60;

    uint8_t servoPin = 18;
    uint8_t servoMinDeg = 40;
    uint8_t servoMaxDeg = 180;
    uint8_t servoManual = 0;
    uint8_t servoManualDeg = 90;
};

struct TelemetryValues
{
    float fsr_ohm = 0.f;
    float flex_avg_ohm = 0.f;
    float servo_deg = 0.f;
};

class BleApp
{
public:
    void begin(const char *name = BLE_DEVICE_NAME);
    void loop();
    void setTelemetry(const TelemetryValues &v);
    const Settings &getSettings() const { return curr; }

private:
    NimBLEServer *server = nullptr;
    NimBLEService *svcTele = nullptr;
    NimBLECharacteristic *chFlex = nullptr, *chServo = nullptr, *chFsr = nullptr;

    NimBLEService *svcCfg = nullptr;
    NimBLECharacteristic *c_fsrPin, *c_fsrPull, *c_fsrStart, *c_fsrMax;
    NimBLECharacteristic *c_flexPin, *c_flexFlat, *c_flexBend;
    NimBLECharacteristic *c_vibPin, *c_vibFreq, *c_vibThr, *c_vibPwr;
    NimBLECharacteristic *c_servoPin, *c_servoMin, *c_servoMax, *c_servoMan, *c_servoManDeg;
    NimBLECharacteristic *c_apply;

    Preferences nvs;
    Settings curr{}, pending{};
    bool havePending = false;

    TelemetryValues lastSent{}, pendingTele{};
    bool haveTele = false;
    uint32_t lastNotifyMs = 0;

    enum class LedMode
    {
        Advertising,
        Connected,
        Notifying
    };
    LedMode ledMode = LedMode::Advertising;
    uint32_t lastBlink = 0;
    void setLed(bool on);
    void updateLed();

    void startAdvertising();
    void buildTelemetry();
    void buildConfig();
    void loadFromNVS();
    void saveToNVS(const Settings &s);
    void installCfgHandlers();

    static void putLEu8(uint8_t v, uint8_t *out);
    static void putLEu16(uint16_t v, uint8_t *out);
    static void putLEu32(uint32_t v, uint8_t *out);

    void applyNow();

public:
    std::function<void(const Settings &)> onReconfigure;
    void onConnect();
    void onDisconnect();
};
