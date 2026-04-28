// Top-level persisted settings model.
#pragma once
#include "settings_types.h"

struct Settings
{
    static constexpr uint32_t kSettingsVersion = 6;

    // Shared FSR configuration used for vibro force feedback.
    uint8_t fsrPin = 33;
    uint32_t fsrPullupOhm = 10000;
    float fsrSoftThresholdN = 7.0f;
    float fsrHardMaxN = 10.0f;

    // Per-pair configuration arrays.
    FlexSettings flex[NUM_PAIRS];
    ServoSettings servo[NUM_PAIRS];
    PairInputSettings pairInput[NUM_PAIRS];
    EmgSettings emg[NUM_PAIRS];

    // Vibro output configuration.
    uint8_t vibroPin = 5;
    VibroMode vibroMode = VibroMode::Normal;
    uint16_t vibroFreqHz = 150;
    uint8_t vibroMaxDuty = 255;
    uint8_t vibroMinDuty = 0;
    uint8_t vibroSoftPower = 200;
    uint8_t vibroPulseBase = 120;

    // Optional BLE authentication code. Zero means no auth is required.
    uint16_t pinCode = 0;

    // Schema/version tag stored with the settings object.
    uint32_t settingsVersion = kSettingsVersion;

    // Build default settings and disconnect unused pairs by default.
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

            emg[i].pin = UNUSED_PIN;
        }
    }

    // Serialize the full settings model into the BLE/NVS JSON shape.
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
        doc["settingsVersion"] = kSettingsVersion;
    }

    // Apply a partial top-level JSON update to the current settings model.
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
                StaticJsonDocument<2048> tmp;
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
                StaticJsonDocument<2048> tmp;
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
                StaticJsonDocument<1024> tmp;
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
                StaticJsonDocument<3072> tmp;
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

        settingsVersion = kSettingsVersion;
    }
};
