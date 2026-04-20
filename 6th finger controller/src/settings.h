#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

static const int NUM_PAIRS = 4;
static const uint8_t UNUSED_PIN = 0xFF;

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

enum class InputSource : uint8_t
{
    Flex = 0,
    Emg = 1
};

enum class EmgMode : uint8_t
{
    BendOther = 0,
    Directional = 1
};

enum class EmgEvent : uint8_t
{
    None = 0,
    Other = 1,
    Bend = 2,
    Unfold = 3
};

enum class EmgAction : uint8_t
{
    None = 0,
    Bend = 1,
    Unfold = 2,
    CooldownIgnored = 3
};

struct FlexSettings
{
    uint8_t flexPin = 32;
    uint32_t flexPullupOhm = 47000;
    uint32_t flexStraightOhm = 65000;
    uint32_t flexBendOhm = 160000;
    uint8_t flexTolerancePct = 5;

    void toJson(JsonVariant dst) const
    {
        dst["flexPin"] = flexPin;
        dst["flexPullupOhm"] = flexPullupOhm;
        dst["flexStraightOhm"] = flexStraightOhm;
        dst["flexBendOhm"] = flexBendOhm;
        dst["flexTolerancePct"] = flexTolerancePct;
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

        if (src.containsKey("flexTolerancePct"))
        {
            int v = src["flexTolerancePct"].as<int>();
            if (v < 1)
                v = 1;
            if (v > 50)
                v = 50;
            flexTolerancePct = (uint8_t)v;
        }
    }
};

struct ServoSettings
{
    uint8_t servoPin = 21;
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

struct PairInputSettings
{
    InputSource inputSource = InputSource::Flex;

    void toJson(JsonVariant dst) const
    {
        dst["inputSource"] = (uint8_t)inputSource;
    }

    void applyJson(JsonVariantConst src)
    {
        if (src.containsKey("inputSource"))
        {
            int v = src["inputSource"].as<int>();
            if (v < (int)InputSource::Flex)
                v = (int)InputSource::Flex;
            if (v > (int)InputSource::Emg)
                v = (int)InputSource::Emg;
            inputSource = (InputSource)v;
        }
    }
};

struct EmgSettings
{
    uint8_t channels = 1;
    uint8_t pin0 = UNUSED_PIN;
    uint8_t pin1 = UNUSED_PIN;
    uint8_t pin2 = UNUSED_PIN;
    EmgMode mode = EmgMode::BendOther;
    uint8_t bendFullMoves = 1;
    uint8_t unfoldFullMoves = 1;
    uint8_t minSwitchDelaySec = 1;
    bool reverseDirection = false;

    void toJson(JsonVariant dst) const
    {
        dst["channels"] = channels;
        JsonArray pins = dst.createNestedArray("pins");
        pins.add(pin0);
        pins.add(pin1);
        pins.add(pin2);
        dst["mode"] = (uint8_t)mode;
        dst["bendFullMoves"] = bendFullMoves;
        dst["unfoldFullMoves"] = unfoldFullMoves;
        dst["minSwitchDelaySec"] = minSwitchDelaySec;
        dst["reverseDirection"] = reverseDirection;
    }

    void applyJson(JsonVariantConst src)
    {
        if (src.containsKey("channels"))
        {
            int v = src["channels"].as<int>();
            if (v < 1)
                v = 1;
            if (v > 3)
                v = 3;
            channels = (uint8_t)v;
        }

        if (src.containsKey("pins"))
        {
            JsonVariantConst pins = src["pins"];
            if (pins.is<JsonArrayConst>())
            {
                JsonArrayConst arr = pins.as<JsonArrayConst>();
                if (arr.size() > 0)
                    pin0 = arr[0];
                if (arr.size() > 1)
                    pin1 = arr[1];
                if (arr.size() > 2)
                    pin2 = arr[2];
            }
        }
        else
        {
            if (src.containsKey("pin0"))
                pin0 = src["pin0"];
            if (src.containsKey("pin1"))
                pin1 = src["pin1"];
            if (src.containsKey("pin2"))
                pin2 = src["pin2"];
        }

        if (src.containsKey("mode"))
        {
            int v = src["mode"].as<int>();
            if (v < (int)EmgMode::BendOther)
                v = (int)EmgMode::BendOther;
            if (v > (int)EmgMode::Directional)
                v = (int)EmgMode::Directional;
            mode = (EmgMode)v;
        }

        if (src.containsKey("bendFullMoves"))
        {
            int v = src["bendFullMoves"].as<int>();
            if (v < 1)
                v = 1;
            if (v > 5)
                v = 5;
            bendFullMoves = (uint8_t)v;
        }

        if (src.containsKey("unfoldFullMoves"))
        {
            int v = src["unfoldFullMoves"].as<int>();
            if (v < 1)
                v = 1;
            if (v > 5)
                v = 5;
            unfoldFullMoves = (uint8_t)v;
        }

        if (src.containsKey("minSwitchDelaySec"))
        {
            int v = src["minSwitchDelaySec"].as<int>();
            if (v < 1)
                v = 1;
            if (v > 60)
                v = 60;
            minSwitchDelaySec = (uint8_t)v;
        }

        if (src.containsKey("reverseDirection"))
            reverseDirection = src["reverseDirection"].as<bool>();
    }

    bool activePinsValid() const
    {
        if (channels < 1 || channels > 3)
            return false;

        const uint8_t pins[3] = {pin0, pin1, pin2};
        for (uint8_t i = 0; i < channels; ++i)
        {
            if (pins[i] == UNUSED_PIN || pins[i] == 0)
                return false;
        }
        return true;
    }
};

struct Settings
{
    uint8_t fsrPin = 33;
    uint32_t fsrPullupOhm = 10000;
    float fsrSoftThresholdN = 7.0f;
    float fsrHardMaxN = 10.0f;

    FlexSettings flex[NUM_PAIRS];
    ServoSettings servo[NUM_PAIRS];
    PairInputSettings pairInput[NUM_PAIRS];
    EmgSettings emg[NUM_PAIRS];

    uint8_t vibroPin = 5;
    VibroMode vibroMode = VibroMode::Normal;
    uint16_t vibroFreqHz = 150;
    uint8_t vibroMaxDuty = 255;
    uint8_t vibroMinDuty = 0;
    uint8_t vibroSoftPower = 200;
    uint8_t vibroPulseBase = 120;

    uint16_t pinCode = 0;

    uint32_t settingsVersion = 2;

    Settings()
    {
        for (int i = 1; i < NUM_PAIRS; ++i)
        {
            flex[i] = FlexSettings{};
            servo[i] = ServoSettings{};
            pairInput[i] = PairInputSettings{};
            emg[i] = EmgSettings{};

            flex[i].flexPin = UNUSED_PIN;
            flex[i].flexPullupOhm = 0;
            flex[i].flexStraightOhm = 0;
            flex[i].flexBendOhm = 0;
            flex[i].flexTolerancePct = 5;

            servo[i].servoPin = UNUSED_PIN;

            pairInput[i].inputSource = InputSource::Flex;

            emg[i].channels = 1;
            emg[i].pin0 = UNUSED_PIN;
            emg[i].pin1 = UNUSED_PIN;
            emg[i].pin2 = UNUSED_PIN;
        }
    }

    void toJson(JsonDocument &doc) const
    {
        doc.clear();

        doc["fsrPin"] = fsrPin;
        doc["fsrPullupOhm"] = fsrPullupOhm;
        doc["fsrSoftThresholdN"] = fsrSoftThresholdN;
        doc["fsrHardMaxN"] = fsrHardMaxN;

        JsonArray flexArr = doc.createNestedArray("flexSettings");
        for (int i = 0; i < NUM_PAIRS; ++i)
        {
            JsonObject obj = flexArr.createNestedObject();
            flex[i].toJson(obj);
        }

        JsonArray servoArr = doc.createNestedArray("servoSettings");
        for (int i = 0; i < NUM_PAIRS; ++i)
        {
            JsonObject obj = servoArr.createNestedObject();
            servo[i].toJson(obj);
        }

        JsonArray pairInputArr = doc.createNestedArray("pairInputSettings");
        for (int i = 0; i < NUM_PAIRS; ++i)
        {
            JsonObject obj = pairInputArr.createNestedObject();
            pairInput[i].toJson(obj);
        }

        JsonArray emgArr = doc.createNestedArray("emgSettings");
        for (int i = 0; i < NUM_PAIRS; ++i)
        {
            JsonObject obj = emgArr.createNestedObject();
            emg[i].toJson(obj);
        }

        doc["vibroPin"] = vibroPin;
        doc["vibroMode"] = (uint8_t)vibroMode;
        doc["vibroFreqHz"] = vibroFreqHz;
        doc["vibroMaxDuty"] = vibroMaxDuty;
        doc["vibroMinDuty"] = vibroMinDuty;
        doc["vibroSoftPower"] = vibroSoftPower;
        doc["vibroPulseBase"] = vibroPulseBase;

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
                JsonArrayConst arr = node.as<JsonArrayConst>();
                for (size_t i = 0; i < NUM_PAIRS && i < arr.size(); ++i)
                {
                    JsonObjectConst obj = arr[i];
                    if (!obj.isNull())
                        flex[i].applyJson(obj);
                }
            }
            else if (node.is<const char *>())
            {
                StaticJsonDocument<1024> tmp;
                auto err = deserializeJson(tmp, node.as<const char *>());
                if (!err && tmp.is<JsonArrayConst>())
                {
                    JsonArrayConst arr = tmp.as<JsonArrayConst>();
                    for (size_t i = 0; i < NUM_PAIRS && i < arr.size(); ++i)
                    {
                        JsonObjectConst obj = arr[i];
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
            if (doc.containsKey("flexTolerancePct"))
            {
                int v = doc["flexTolerancePct"].as<int>();
                if (v < 1)
                    v = 1;
                if (v > 50)
                    v = 50;
                flex[0].flexTolerancePct = (uint8_t)v;
            }
        }

        if (doc.containsKey("servoSettings"))
        {
            JsonVariantConst node = doc["servoSettings"];

            if (node.is<JsonArrayConst>())
            {
                JsonArrayConst arr = node.as<JsonArrayConst>();
                for (size_t i = 0; i < NUM_PAIRS && i < arr.size(); ++i)
                {
                    JsonObjectConst obj = arr[i];
                    if (!obj.isNull())
                        servo[i].applyJson(obj);
                }
            }
            else if (node.is<const char *>())
            {
                StaticJsonDocument<1024> tmp;
                auto err = deserializeJson(tmp, node.as<const char *>());
                if (!err && tmp.is<JsonArrayConst>())
                {
                    JsonArrayConst arr = tmp.as<JsonArrayConst>();
                    for (size_t i = 0; i < NUM_PAIRS && i < arr.size(); ++i)
                    {
                        JsonObjectConst obj = arr[i];
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

        if (doc.containsKey("pairInputSettings"))
        {
            JsonVariantConst node = doc["pairInputSettings"];

            if (node.is<JsonArrayConst>())
            {
                JsonArrayConst arr = node.as<JsonArrayConst>();
                for (size_t i = 0; i < NUM_PAIRS && i < arr.size(); ++i)
                {
                    JsonObjectConst obj = arr[i];
                    if (!obj.isNull())
                        pairInput[i].applyJson(obj);
                }
            }
            else if (node.is<const char *>())
            {
                StaticJsonDocument<512> tmp;
                auto err = deserializeJson(tmp, node.as<const char *>());
                if (!err && tmp.is<JsonArrayConst>())
                {
                    JsonArrayConst arr = tmp.as<JsonArrayConst>();
                    for (size_t i = 0; i < NUM_PAIRS && i < arr.size(); ++i)
                    {
                        JsonObjectConst obj = arr[i];
                        if (!obj.isNull())
                            pairInput[i].applyJson(obj);
                    }
                }
            }
        }

        if (doc.containsKey("emgSettings"))
        {
            JsonVariantConst node = doc["emgSettings"];

            if (node.is<JsonArrayConst>())
            {
                JsonArrayConst arr = node.as<JsonArrayConst>();
                for (size_t i = 0; i < NUM_PAIRS && i < arr.size(); ++i)
                {
                    JsonObjectConst obj = arr[i];
                    if (!obj.isNull())
                        emg[i].applyJson(obj);
                }
            }
            else if (node.is<const char *>())
            {
                StaticJsonDocument<1024> tmp;
                auto err = deserializeJson(tmp, node.as<const char *>());
                if (!err && tmp.is<JsonArrayConst>())
                {
                    JsonArrayConst arr = tmp.as<JsonArrayConst>();
                    for (size_t i = 0; i < NUM_PAIRS && i < arr.size(); ++i)
                    {
                        JsonObjectConst obj = arr[i];
                        if (!obj.isNull())
                            emg[i].applyJson(obj);
                    }
                }
            }
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
