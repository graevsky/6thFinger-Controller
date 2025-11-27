#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

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

struct Settings
{

    uint8_t fsrPin = 33;
    uint32_t fsrPullupOhm = 10000;
    float fsrSoftThresholdN = 7.0f;
    float fsrHardMaxN = 10.0f;

    uint8_t flexPin = 32;
    uint32_t flexPullupOhm = 47000;
    uint32_t flexStraightOhm = 65000;
    uint32_t flexBendOhm = 160000;

    uint8_t vibroPin = 5;
    VibroMode vibroMode = VibroMode::Normal;
    uint16_t vibroFreqHz = 150;
    uint8_t vibroMaxDuty = 255;
    uint8_t vibroMinDuty = 0;
    uint8_t vibroSoftPower = 200;
    uint8_t vibroPulseBase = 120;

    uint8_t servoPin = 18;
    uint8_t servoMinDeg = 40;
    uint8_t servoMaxDeg = 180;
    ServoManualMode servoManual = ServoManualMode::Auto;
    uint8_t servoManualDeg = 90;
    float servoMaxSpeedDegPerSec = 300.0f;

    uint32_t settingsVersion = 1;

    void toJson(JsonDocument &doc) const
    {
        doc["fsrPin"] = fsrPin;
        doc["fsrPullupOhm"] = fsrPullupOhm;
        doc["fsrSoftThresholdN"] = fsrSoftThresholdN;
        doc["fsrHardMaxN"] = fsrHardMaxN;

        doc["flexPin"] = flexPin;
        doc["flexPullupOhm"] = flexPullupOhm;
        doc["flexStraightOhm"] = flexStraightOhm;
        doc["flexBendOhm"] = flexBendOhm;

        doc["vibroPin"] = vibroPin;
        doc["vibroMode"] = (uint8_t)vibroMode;
        doc["vibroFreqHz"] = vibroFreqHz;
        doc["vibroMaxDuty"] = vibroMaxDuty;
        doc["vibroMinDuty"] = vibroMinDuty;
        doc["vibroSoftPower"] = vibroSoftPower;
        doc["vibroPulseBase"] = vibroPulseBase;

        doc["servoPin"] = servoPin;
        doc["servoMinDeg"] = servoMinDeg;
        doc["servoMaxDeg"] = servoMaxDeg;
        doc["servoManual"] = (uint8_t)servoManual;
        doc["servoManualDeg"] = servoManualDeg;
        doc["servoMaxSpeedDegPerSec"] = servoMaxSpeedDegPerSec;

        doc["settingsVersion"] = settingsVersion;
    }

    void applyJson(const JsonDocument &doc)
    {
        if (doc.containsKey("fsrPin"))
            fsrPin = doc["fsrPin"];
        if (doc.containsKey("fsrPullupOhm"))
            fsrPullupOhm = doc["fsrPullupOhm"];
        if (doc.containsKey("fsrSoftThresholdN"))
            fsrSoftThresholdN = doc["fsrSoftThresholdN"];
        if (doc.containsKey("fsrHardMaxN"))
            fsrHardMaxN = doc["fsrHardMaxN"];

        if (doc.containsKey("flexPin"))
            flexPin = doc["flexPin"];
        if (doc.containsKey("flexPullupOhm"))
            flexPullupOhm = doc["flexPullupOhm"];
        if (doc.containsKey("flexStraightOhm"))
            flexStraightOhm = doc["flexStraightOhm"];
        if (doc.containsKey("flexBendOhm"))
            flexBendOhm = doc["flexBendOhm"];

        if (doc.containsKey("vibroPin"))
            vibroPin = doc["vibroPin"];
        if (doc.containsKey("vibroMode"))
            vibroMode = (VibroMode)((uint8_t)doc["vibroMode"]);
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

        if (doc.containsKey("servoPin"))
            servoPin = doc["servoPin"];
        if (doc.containsKey("servoMinDeg"))
            servoMinDeg = doc["servoMinDeg"];
        if (doc.containsKey("servoMaxDeg"))
            servoMaxDeg = doc["servoMaxDeg"];
        if (doc.containsKey("servoManual"))
            servoManual = (ServoManualMode)((uint8_t)doc["servoManual"]);
        if (doc.containsKey("servoManualDeg"))
            servoManualDeg = doc["servoManualDeg"];
        if (doc.containsKey("servoMaxSpeedDegPerSec"))
            servoMaxSpeedDegPerSec = doc["servoMaxSpeedDegPerSec"];

        if (doc.containsKey("settingsVersion"))
            settingsVersion = doc["settingsVersion"];
    }
};
