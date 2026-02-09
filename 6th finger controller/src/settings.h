#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

static const int NUM_PAIRS = 4;

enum class VibroMode : uint8_t
{
    Normal = 0,
    Pulse = 1
};

enum class ServoManualMode : uint8_t
{
    Auto = 0,
    Manual = 1
};

struct FlexSettings
{
    uint8_t flexPin = 32;
    uint32_t flexPullupOhm = 47000;
    uint32_t flexStraightOhm = 65000;
    uint32_t flexBendOhm = 160000;

    void toJson(JsonVariant dst) const
    {
        dst["flexPin"] = flexPin;
        dst["flexPullupOhm"] = flexPullupOhm;
        dst["flexStraightOhm"] = flexStraightOhm;
        dst["flexBendOhm"] = flexBendOhm;
    }

    void applyJson(JsonVariantConst src)
    {
        if (src.containsKey("flexPin"))
            flexPin = src["flexPin"];
        if (src.containsKey("flexPullupOhm"))
            flexPullupOhm = src["flexPullupOhm"];
        if (src.containsKey("flexStraightOhm"))
            flexStraightOhm = src["flexStraightOhm"];
        if (src.containsKey("flexBendOhm"))
            flexBendOhm = src["flexBendOhm"];
    }
};

struct ServoSettings
{
    uint8_t servoPin = 20;
    uint8_t servoMinDeg = 40;
    uint8_t servoMaxDeg = 180;
    ServoManualMode servoManual = ServoManualMode::Auto;
    uint8_t servoManualDeg = 90;
    float servoMaxSpeedDegPerSec = 300.0f;

    void toJson(JsonVariant dst) const
    {
        dst["servoPin"] = servoPin;
        dst["servoMinDeg"] = servoMinDeg;
        dst["servoMaxDeg"] = servoMaxDeg;
        dst["servoManual"] = (uint8_t)servoManual;
        dst["servoManualDeg"] = servoManualDeg;
        dst["servoMaxSpeedDegPerSec"] = servoMaxSpeedDegPerSec;
    }

    void applyJson(JsonVariantConst src)
    {
        if (src.containsKey("servoPin"))
            servoPin = src["servoPin"];
        if (src.containsKey("servoMinDeg"))
            servoMinDeg = src["servoMinDeg"];
        if (src.containsKey("servoMaxDeg"))
            servoMaxDeg = src["servoMaxDeg"];
        if (src.containsKey("servoManual"))
            servoManual = (ServoManualMode)((uint8_t)src["servoManual"].as<uint8_t>());
        if (src.containsKey("servoManualDeg"))
            servoManualDeg = src["servoManualDeg"];
        if (src.containsKey("servoMaxSpeedDegPerSec"))
            servoMaxSpeedDegPerSec = src["servoMaxSpeedDegPerSec"];
    }
};

struct Settings
{
    /// FSR
    uint8_t fsrPin = 33;
    uint32_t fsrPullupOhm = 10000;
    float fsrSoftThresholdN = 7.0f;
    float fsrHardMaxN = 10.0f;

    // flex & servo pairs
    FlexSettings flex[NUM_PAIRS];
    ServoSettings servo[NUM_PAIRS];

    // Vibro
    uint8_t vibroPin = 5;
    VibroMode vibroMode = VibroMode::Normal;
    uint16_t vibroFreqHz = 150;
    uint8_t vibroMaxDuty = 255;
    uint8_t vibroMinDuty = 0;
    uint8_t vibroSoftPower = 200;
    uint8_t vibroPulseBase = 120;

    // PIN (0000 = выключен)
    uint16_t pinCode = 0;

    uint32_t settingsVersion = 1;

    Settings()
    {
        // Flex & Servo settings. Pairs 2-4 are inactive by default
        for (int i = 1; i < NUM_PAIRS; ++i)
        {
            flex[i] = FlexSettings{};
            servo[i] = ServoSettings{};

            flex[i].flexPin = 0xFF;
            flex[i].flexPullupOhm = 0;
            flex[i].flexStraightOhm = 0;
            flex[i].flexBendOhm = 0;

            servo[i].servoPin = 0xFF;
        }
    }

    void toJson(JsonDocument &doc) const
    {
        doc.clear();

        // FSR
        doc["fsrPin"] = fsrPin;
        doc["fsrPullupOhm"] = fsrPullupOhm;
        doc["fsrSoftThresholdN"] = fsrSoftThresholdN;
        doc["fsrHardMaxN"] = fsrHardMaxN;

        // flexSettings
        JsonArray flexArr = doc.createNestedArray("flexSettings");
        for (int i = 0; i < NUM_PAIRS; ++i)
        {
            JsonObject obj = flexArr.createNestedObject();
            flex[i].toJson(obj);
        }

        // servoSettings
        JsonArray servoArr = doc.createNestedArray("servoSettings");
        for (int i = 0; i < NUM_PAIRS; ++i)
        {
            JsonObject obj = servoArr.createNestedObject();
            servo[i].toJson(obj);
        }

        // Vibro
        doc["vibroPin"] = vibroPin;
        doc["vibroMode"] = (uint8_t)vibroMode;
        doc["vibroFreqHz"] = vibroFreqHz;
        doc["vibroMaxDuty"] = vibroMaxDuty;
        doc["vibroMinDuty"] = vibroMinDuty;
        doc["vibroSoftPower"] = vibroSoftPower;
        doc["vibroPulseBase"] = vibroPulseBase;

        // PIN
        doc["pinCode"] = pinCode;

        doc["settingsVersion"] = settingsVersion;
    }

    void applyJson(const JsonDocument &doc)
    {
        using namespace ArduinoJson;

        if (doc.containsKey("fsrPin"))
            fsrPin = doc["fsrPin"];
        if (doc.containsKey("fsrPullupOhm"))
            fsrPullupOhm = doc["fsrPullupOhm"];
        if (doc.containsKey("fsrSoftThresholdN"))
            fsrSoftThresholdN = doc["fsrSoftThresholdN"];
        if (doc.containsKey("fsrHardMaxN"))
            fsrHardMaxN = doc["fsrHardMaxN"];

        if (doc.containsKey("flexSettings"))
        {
            JsonVariantConst node = doc["flexSettings"];

            if (node.is<JsonArrayConst>())
            {
                JsonArrayConst flexArr = node.as<JsonArrayConst>();
                for (size_t i = 0; i < NUM_PAIRS && i < flexArr.size(); ++i)
                {
                    JsonObjectConst obj = flexArr[i];
                    if (!obj.isNull())
                        flex[i].applyJson(obj);
                }
            }
            else if (node.is<const char *>())
            {
                StaticJsonDocument<512> tmp;
                auto err = deserializeJson(tmp, node.as<const char *>());
                if (!err && tmp.is<JsonArrayConst>())
                {
                    JsonArrayConst flexArr = tmp.as<JsonArrayConst>();
                    for (size_t i = 0; i < NUM_PAIRS && i < flexArr.size(); ++i)
                    {
                        JsonObjectConst obj = flexArr[i];
                        if (!obj.isNull())
                            flex[i].applyJson(obj);
                    }
                }
            }
        }
        else
        {
            if (doc.containsKey("flexPin"))
                flex[0].flexPin = doc["flexPin"];
            if (doc.containsKey("flexPullupOhm"))
                flex[0].flexPullupOhm = doc["flexPullupOhm"];
            if (doc.containsKey("flexStraightOhm"))
                flex[0].flexStraightOhm = doc["flexStraightOhm"];
            if (doc.containsKey("flexBendOhm"))
                flex[0].flexBendOhm = doc["flexBendOhm"];
        }

        if (doc.containsKey("servoSettings"))
        {
            JsonVariantConst node = doc["servoSettings"];

            if (node.is<JsonArrayConst>())
            {
                JsonArrayConst servoArr = node.as<JsonArrayConst>();
                for (size_t i = 0; i < NUM_PAIRS && i < servoArr.size(); ++i)
                {
                    JsonObjectConst obj = servoArr[i];
                    if (!obj.isNull())
                        servo[i].applyJson(obj);
                }
            }
            else if (node.is<const char *>())
            {
                StaticJsonDocument<512> tmp;
                auto err = deserializeJson(tmp, node.as<const char *>());
                if (!err && tmp.is<JsonArrayConst>())
                {
                    JsonArrayConst servoArr = tmp.as<JsonArrayConst>();
                    for (size_t i = 0; i < NUM_PAIRS && i < servoArr.size(); ++i)
                    {
                        JsonObjectConst obj = servoArr[i];
                        if (!obj.isNull())
                            servo[i].applyJson(obj);
                    }
                }
            }
        }
        else
        {
            if (doc.containsKey("servoPin"))
                servo[0].servoPin = doc["servoPin"];
            if (doc.containsKey("servoMinDeg"))
                servo[0].servoMinDeg = doc["servoMinDeg"];
            if (doc.containsKey("servoMaxDeg"))
                servo[0].servoMaxDeg = doc["servoMaxDeg"];
            if (doc.containsKey("servoManual"))
                servo[0].servoManual =
                    (ServoManualMode)((uint8_t)doc["servoManual"].as<uint8_t>());
            if (doc.containsKey("servoManualDeg"))
                servo[0].servoManualDeg = doc["servoManualDeg"];
            if (doc.containsKey("servoMaxSpeedDegPerSec"))
                servo[0].servoMaxSpeedDegPerSec = doc["servoMaxSpeedDegPerSec"];
        }

        if (doc.containsKey("vibroPin"))
            vibroPin = doc["vibroPin"];
        if (doc.containsKey("vibroMode"))
            vibroMode = (VibroMode)((uint8_t)doc["vibroMode"].as<uint8_t>());
        if (doc.containsKey("vibroFreqHz"))
            vibroFreqHz = doc["vibroFreqHz"];
        if (doc.containsKey("vibroMaxDuty"))
            vibroMaxDuty = doc["vibroMaxDuty"];
        if (doc.containsKey("vibroMinDuty"))
            vibroMinDuty = doc["vibroMinDuty"];
        if (doc.containsKey("vibroSoftPower"))
            vibroSoftPower = doc["vibroSoftPower"];
        if (doc.containsKey("vibroPulseBase"))
            vibroPulseBase = doc["vibroPulseBase"];

        if (doc.containsKey("pinCode"))
            pinCode = doc["pinCode"];

        if (doc.containsKey("settingsVersion"))
            settingsVersion = doc["settingsVersion"];
    }
};
