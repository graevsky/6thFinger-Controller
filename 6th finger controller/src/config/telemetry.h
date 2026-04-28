// Shared telemetry snapshot used by the control loop and BLE transport.
#pragma once
#include <Arduino.h>
#include "settings_types.h"

struct ControlTelemetry
{
    // Latest raw and filtered flex resistance values for each pair.
    float flexRawOhm[NUM_PAIRS] = {};
    float flexFilteredOhm[NUM_PAIRS] = {};

    // Latest FSR resistance and force estimate.
    float fsrRawOhm = 0.0f;
    float fsrFilteredOhm = 0.0f;
    float fsrForceN = 0.0f;

    // Servo command, current position, and estimated speed.
    float servoTargetDeg[NUM_PAIRS] = {};
    float servoCurrentDeg[NUM_PAIRS] = {};
    float servoSpeedDps[NUM_PAIRS] = {};

    // EMG classifier output, cooldown state, and the latest raw input sample.
    int8_t emgSource[NUM_PAIRS] = {};
    int8_t emgEvent[NUM_PAIRS] = {};
    int8_t emgAction[NUM_PAIRS] = {};
    int16_t emgCooldownMs[NUM_PAIRS] = {};
    int8_t emgBendProgress[NUM_PAIRS] = {};
    int8_t emgUnfoldProgress[NUM_PAIRS] = {};
    float emgCh0[NUM_PAIRS] = {};

    // Vibro output state derived from the latest FSR force.
    uint8_t vibroDuty = 0;
    bool vibroActive = false;
    VibroMode vibroMode = VibroMode::Normal;
};
