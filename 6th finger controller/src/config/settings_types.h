// Core configuration types shared across the firmware.
#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

static const int NUM_PAIRS = 5;
static const uint8_t UNUSED_PIN = 0xFF;

// Vibro output style above the soft force threshold.
enum class VibroMode : uint8_t
{
    Normal = 0,
    Pulse = 1
};

// Whether a servo follows sensor input or a forced manual angle.
enum class ServoManualMode : uint8_t
{
    Auto = 0,
    Manual = 1
};

// Input source assigned to one pair.
enum class InputSource : uint8_t
{
    Flex = 0,
    Emg = 1
};

// Low-level EMG event visible in telemetry.
enum class EmgEvent : uint8_t
{
    None = 0,
    Snapshot = 1
};

// High-level action emitted by the EMG state machine.
enum class EmgAction : uint8_t
{
    None = 0,
    Bend = 1,
    Unfold = 2,
    CooldownIgnored = 3
};

// Configuration for one flex sensor channel.
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

// Configuration for one servo output.
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

// Selects whether one logical pair is driven by flex or EMG.
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

// Configuration for one EMG-driven pair using only the 1-channel binary model.
struct EmgSettings
{
    uint8_t pin = 34;
    uint8_t bendSnapshotsToBend = 1;
    uint8_t bendSnapshotsToUnfold = 1;
    uint8_t snapshotTimeoutSec = 5;
    uint8_t snapshotSize = 2;
    uint8_t minUnfoldDelaySec = 2;
    bool reverseDirection = false;

    void toJson(JsonVariant dst) const
    {
        JsonArray pins = dst.createNestedArray("pins");
        pins.add(pin);
        dst["bendSnapshotsToBend"] = bendSnapshotsToBend;
        dst["bendSnapshotsToUnfold"] = bendSnapshotsToUnfold;
        dst["snapshotTimeoutSec"] = snapshotTimeoutSec;
        dst["snapshotSize"] = snapshotSize;
        dst["minUnfoldDelaySec"] = minUnfoldDelaySec;
        dst["reverseDirection"] = reverseDirection;
    }

    void applyJson(JsonVariantConst src)
    {
        if (src.containsKey("pins"))
        {
            JsonVariantConst pins = src["pins"];
            if (pins.is<JsonArrayConst>())
            {
                JsonArrayConst arr = pins.as<JsonArrayConst>();
                if (arr.size() > 0)
                    pin = arr[0];
            }
        }

        if (src.containsKey("bendSnapshotsToBend"))
        {
            int v = src["bendSnapshotsToBend"].as<int>();
            if (v < 1)
                v = 1;
            if (v > 5)
                v = 5;
            bendSnapshotsToBend = (uint8_t)v;
        }

        if (src.containsKey("bendSnapshotsToUnfold"))
        {
            int v = src["bendSnapshotsToUnfold"].as<int>();
            if (v < 1)
                v = 1;
            if (v > 8)
                v = 8;
            bendSnapshotsToUnfold = (uint8_t)v;
        }

        if (src.containsKey("snapshotTimeoutSec"))
        {
            int v = src["snapshotTimeoutSec"].as<int>();
            if (v < 1)
                v = 1;
            if (v > 15)
                v = 15;
            snapshotTimeoutSec = (uint8_t)v;
        }

        if (src.containsKey("snapshotSize"))
        {
            int v = src["snapshotSize"].as<int>();
            if (v < 1)
                v = 1;
            if (v > 32)
                v = 32;
            snapshotSize = (uint8_t)v;
        }

        if (src.containsKey("minUnfoldDelaySec"))
        {
            int v = src["minUnfoldDelaySec"].as<int>();
            if (v < 0)
                v = 0;
            if (v > 30)
                v = 30;
            minUnfoldDelaySec = (uint8_t)v;
        }

        if (src.containsKey("reverseDirection"))
            reverseDirection = src["reverseDirection"].as<bool>();
    }

    bool activePinsValid() const
    {
        return pin != UNUSED_PIN && pin != 0;
    }
};
