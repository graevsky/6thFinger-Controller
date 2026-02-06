#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
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

    uint8_t vibroDuty = 0;
    bool vibroActive = false;
    VibroMode vibroMode = VibroMode::Normal;
};

class Control
{
public:
    Control() = default;

    void begin(const Settings &s);
    void reconfigure(const Settings &s);
    void update();

    const ControlTelemetry &getTelemetry() const { return tele; }
    const Settings &getSettings() const { return current; }

private:
    Settings current{};

    Servo servos[NUM_PAIRS];
    float servoAngleDeg[NUM_PAIRS] = {};
    uint32_t lastServoUpdate[NUM_PAIRS] = {};

    float flexFiltered[NUM_PAIRS] = {};
    float fsrFiltered = 0.0f;

    float flexRaw[NUM_PAIRS] = {};
    float fsrRaw = 0.0f;

    uint8_t vibroDuty = 0;

    ControlTelemetry tele{};

    float smooth(float prev, float cur, float alpha);
    float readResistance(uint8_t pin, uint32_t pullupOhm);
    float fsrToNewton(float resistanceOhm);
    float flexToAngle(float rohm, int idx) const;

    uint8_t computeVibroDuty(float forceN, bool &outPulseMode);

    void updateServo(float targetAngleDeg, int idx);
    void updateVibro(uint8_t duty);
    void setupHardware();
};
