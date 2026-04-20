#pragma once
#include <Arduino.h>
#include "settings.h"

struct ControlTelemetry
{
    float flexRawOhm[NUM_PAIRS] = {};
    float flexFilteredOhm[NUM_PAIRS] = {};

    float fsrRawOhm = 0.0f;
    float fsrFilteredOhm = 0.0f;
    float fsrForceN = 0.0f;

    float servoTargetDeg[NUM_PAIRS] = {};
    float servoCurrentDeg[NUM_PAIRS] = {};
    float servoSpeedDps[NUM_PAIRS] = {};

    int8_t emgSource[NUM_PAIRS] = {};
    int8_t emgMode[NUM_PAIRS] = {};
    int8_t emgChannelCount[NUM_PAIRS] = {};
    int8_t emgEvent[NUM_PAIRS] = {};
    int8_t emgAction[NUM_PAIRS] = {};
    int16_t emgCooldownMs[NUM_PAIRS] = {};
    int8_t emgBendProgress[NUM_PAIRS] = {};
    int8_t emgUnfoldProgress[NUM_PAIRS] = {};
    float emgCh0[NUM_PAIRS] = {};
    float emgCh1[NUM_PAIRS] = {};
    float emgCh2[NUM_PAIRS] = {};

    uint8_t vibroDuty = 0;
    bool vibroActive = false;
    VibroMode vibroMode = VibroMode::Normal;
};
