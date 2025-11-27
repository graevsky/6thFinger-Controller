#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "settings.h"

struct ControlTelemetry
{

    float flexRawOhm = 0.0f;
    float flexFilteredOhm = 0.0f;

    float fsrRawOhm = 0.0f;
    float fsrFilteredOhm = 0.0f;
    float fsrForceN = 0.0f;

    float servoTargetDeg = 0.0f;
    float servoCurrentDeg = 0.0f;
    float servoSpeedDps = 0.0f;

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

    Servo servo;
    float servoAngleDeg = 180.0f;
    uint32_t lastServoUpdate = 0;

    float flexFiltered = 0.0f;
    float fsrFiltered = 0.0f;

    float flexRaw = 0.0f;
    float fsrRaw = 0.0f;

    uint8_t vibroDuty = 0;

    ControlTelemetry tele{};

    float smooth(float prev, float cur, float alpha);
    float readResistance(uint8_t pin, uint32_t pullupOhm);
    float fsrToNewton(float resistanceOhm);
    float flexToAngle(float rohm) const;

    uint8_t computeVibroDuty(float forceN, bool &outPulseMode);

    void updateServo(float targetAngleDeg);
    void updateVibro(uint8_t duty);
    void setupHardware();
};
