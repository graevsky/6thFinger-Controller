// Control subsystem public interface. It owns sensor acquisition, EMG
// inference, vibro output, and per-pair servo control.
#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "../config/settings.h"
#include "../config/telemetry.h"
#include "../emg/emg_engine.h"

class Control
{
public:
    Control() = default;

    // Initialize the control subsystem with a settings snapshot.
    void begin(const Settings &s);
    // Reinitialize hardware and internal state after settings change.
    void reconfigure(const Settings &s);
    // Execute one control loop iteration.
    void update();

    // Expose the latest telemetry snapshot built by the control loop.
    const ControlTelemetry &getTelemetry() const { return tele; }
    // Expose the currently applied settings snapshot.
    const Settings &getSettings() const { return current; }

    // Temporarily override one servo target from BLE live-control commands.
    void liveServoSet(int idx, float deg);
    // Cancel the live-control override for one servo.
    void liveServoStop(int idx);

private:
    // Active settings snapshot currently applied to hardware.
    Settings current{};

    // Per-pair servo objects and their motion state.
    Servo servos[NUM_PAIRS];
    float servoAngleDeg[NUM_PAIRS] = {};
    float targetAngleDeg[NUM_PAIRS] = {};
    uint32_t lastServoUpdate[NUM_PAIRS] = {};

    // Latest filtered sensor values used by the control loop.
    float flexFiltered[NUM_PAIRS] = {};
    float fsrFiltered = 0.0f;

    // Latest raw sensor values kept for telemetry and filtering.
    float flexRaw[NUM_PAIRS] = {};
    float fsrRaw = 0.0f;

    // Low-pass filtered FSR ADC value used before resistance conversion.
    float fsrAdcFiltered = 0.0f;

    // Last PWM duty written to the vibro channel.
    uint8_t vibroDuty = 0;

    // Shared telemetry snapshot and EMG engine instance.
    ControlTelemetry tele{};
    EmgEngine emg;

    // FSR acquisition and conversion constants.
    static constexpr int FSR_SAMPLES = 8;
    static constexpr int FSR_OPEN_ADC = 6;
    static constexpr float FSR_MAX_REPORT_OHM = 10000000.0f;
    static constexpr float FSR_PRESS_ALPHA = 0.55f;
    static constexpr float FSR_RELEASE_ALPHA = 0.35f;

    // Flex acquisition settings.
    static constexpr int FLEX_SAMPLES = 8;

    // ADC settle configuration used when switching analog channels.
    static constexpr int ADC_DUMMY_READS = 3;
    static constexpr int ADC_SETTLE_US = 280;
    uint8_t lastAdcPin = 0xFF;

    // Flex history used for simple outlier rejection.
    static constexpr int FLEX_HISTORY = 5;
    float flexHist[NUM_PAIRS][FLEX_HISTORY] = {};
    uint8_t flexHistCount[NUM_PAIRS] = {};
    uint8_t flexHistPos[NUM_PAIRS] = {};
    uint8_t flexOutlierStrikes[NUM_PAIRS] = {};

    float flexStableOhm[NUM_PAIRS] = {};
    bool flexStableInit[NUM_PAIRS] = {};

    // Live control TTL and round-robin scheduling state for slow sensors.
    static constexpr uint32_t LIVE_TTL_MS = 1200;
    static constexpr uint32_t FLEX_SENSOR_PERIOD_MS = 25;
    static constexpr uint32_t FSR_SENSOR_PERIOD_MS = 25;
    bool liveActive[NUM_PAIRS] = {};
    uint32_t liveUntilMs[NUM_PAIRS] = {};
    float liveDeg[NUM_PAIRS] = {};
    uint32_t lastFlexSensorUpdateMs[NUM_PAIRS] = {};
    uint32_t lastFsrSensorUpdateMs = 0;
    uint8_t slowSensorCursor = 0;

private:
    // Bridge callback passed into the EMG engine so it can reuse stable ADC reads.
    static int adcReadBridge(void *ctx, uint8_t pin, int samples);

    // Treat 0 and UNUSED_PIN as disconnected pins.
    bool isValidPin(uint8_t pin) const;

    // Shared first-order smoothing helper.
    float smooth(float prev, float cur, float alpha);

    // Stable ADC and resistance helpers used by FSR, flex, and EMG.
    int readAdcAvgStable(uint8_t pin, int samples);
    float readResistanceStable(uint8_t pin, uint32_t pullupOhm, int samples);

    // Sensor-domain conversion helpers.
    float resistanceFromAdc(float adc, uint32_t pullupOhm) const;
    float sanitizeResistanceForTelemetry(float resistanceOhm) const;

    float fsrToNewton(float resistanceOhm);
    float flexToAngle(float rohm, int idx) const;

    // Vibro mapping from force to PWM duty.
    uint8_t computeVibroDuty(float forceN, bool &outPulseMode);

    // Low-level actuator and sensor update helpers.
    void updateServo(float targetAngleDeg, int idx);
    void updateVibro(uint8_t duty);
    void setupHardware();
    void pauseVibroForAdc();
    void restoreVibroAfterAdc();
    void updateFlexInput(int idx);
    void updateFsrInput(uint32_t nowMs);
    void processOneSlowInput(uint32_t nowMs);

    // Flex history utilities used by the outlier filter.
    void pushFlexHistory(int idx, float v);
    float medianFlexHistory(int idx) const;
};
