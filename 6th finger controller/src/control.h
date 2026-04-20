#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "settings.h"
#include "telemetry.h"
#include "emg_engine.h"

class Control
{
public:
    Control() = default;

    void begin(const Settings &s);
    void reconfigure(const Settings &s);
    void update();

    const ControlTelemetry &getTelemetry() const { return tele; }
    const Settings &getSettings() const { return current; }

    void liveServoSet(int idx, float deg);
    void liveServoStop(int idx);

private:
    Settings current{};

    Servo servos[NUM_PAIRS];
    float servoAngleDeg[NUM_PAIRS] = {};
    uint32_t lastServoUpdate[NUM_PAIRS] = {};

    float flexFiltered[NUM_PAIRS] = {};
    float fsrFiltered = 0.0f;

    float flexRaw[NUM_PAIRS] = {};
    float fsrRaw = 0.0f;

    float fsrAdcFiltered = 0.0f;

    uint8_t vibroDuty = 0;

    ControlTelemetry tele{};
    EmgEngine emg;

    static constexpr int FSR_SAMPLES = 8;
    static constexpr int FSR_OPEN_ADC = 6;
    static constexpr float FSR_MAX_REPORT_OHM = 10000000.0f;
    static constexpr float FSR_PRESS_ALPHA = 0.55f;
    static constexpr float FSR_RELEASE_ALPHA = 0.35f;

    static constexpr int FLEX_SAMPLES = 8;

    static constexpr int ADC_DUMMY_READS = 3;
    static constexpr int ADC_SETTLE_US = 280;
    uint8_t lastAdcPin = 0xFF;

    static constexpr int FLEX_HISTORY = 5;
    float flexHist[NUM_PAIRS][FLEX_HISTORY] = {};
    uint8_t flexHistCount[NUM_PAIRS] = {};
    uint8_t flexHistPos[NUM_PAIRS] = {};
    uint8_t flexOutlierStrikes[NUM_PAIRS] = {};

    float flexStableOhm[NUM_PAIRS] = {};
    bool flexStableInit[NUM_PAIRS] = {};

    static constexpr uint32_t LIVE_TTL_MS = 1200;
    bool liveActive[NUM_PAIRS] = {};
    uint32_t liveUntilMs[NUM_PAIRS] = {};
    float liveDeg[NUM_PAIRS] = {};

private:
    static int adcReadBridge(void *ctx, uint8_t pin, int samples);

    bool isValidPin(uint8_t pin) const;

    float smooth(float prev, float cur, float alpha);

    int readAdcAvgStable(uint8_t pin, int samples);
    float readResistanceStable(uint8_t pin, uint32_t pullupOhm, int samples);

    float resistanceFromAdc(float adc, uint32_t pullupOhm) const;
    float sanitizeResistanceForTelemetry(float resistanceOhm) const;

    float fsrToNewton(float resistanceOhm);
    float flexToAngle(float rohm, int idx) const;

    uint8_t computeVibroDuty(float forceN, bool &outPulseMode);

    void updateServo(float targetAngleDeg, int idx);
    void updateVibro(uint8_t duty);
    void setupHardware();

    void pushFlexHistory(int idx, float v);
    float medianFlexHistory(int idx) const;
};