// Vibro/FSR helpers.
#include "control.h"

#include <math.h>

namespace
{
    static constexpr int VIBRO_CHANNEL = 4;

    float clampf(float v, float a, float b)
    {
        if (v < a)
            return a;
        if (v > b)
            return b;
        return v;
    }
}

// Convert the estimated force into a vibro PWM duty and optional pulse behavior.
uint8_t Control::computeVibroDuty(float N, bool &outPulseMode)
{
    outPulseMode = false;

    if (N <= 0.0f)
        return 0;

    if (N <= current.fsrSoftThresholdN)
    {
        float t = N / current.fsrSoftThresholdN;
        t = clampf(t, 0.0f, 1.0f);

        uint8_t duty = (uint8_t)(t * current.vibroSoftPower);
        if (duty < current.vibroMinDuty)
            duty = current.vibroMinDuty;
        if (duty > current.vibroMaxDuty)
            duty = current.vibroMaxDuty;
        return duty;
    }

    outPulseMode = true;

    float over = N - current.fsrSoftThresholdN;
    float span = current.fsrHardMaxN - current.fsrSoftThresholdN;
    if (span <= 0.0f)
        span = 1.0f;

    float t = over / span;
    t = clampf(t, 0.0f, 1.0f);

    float base = current.vibroPulseBase;
    float amp = t * (current.vibroMaxDuty - base);

    float x = millis() / 1000.0f;
    float dutyF = base;

    // Above the soft threshold, pulse mode uses a sine envelope for stronger feedback.
    if (current.vibroMode == VibroMode::Pulse)
        dutyF = base + sinf(x * 2.0f * PI * 4.0f) * amp;
    else
        dutyF = base + amp;

    if (dutyF < current.vibroMinDuty)
        dutyF = current.vibroMinDuty;
    if (dutyF > current.vibroMaxDuty)
        dutyF = current.vibroMaxDuty;

    return (uint8_t)dutyF;
}

// Commit the vibro duty to hardware and mirror it into telemetry.
void Control::updateVibro(uint8_t duty)
{
    vibroDuty = duty;
    tele.vibroDuty = duty;
    tele.vibroActive = duty > 0;
    tele.vibroMode = current.vibroMode;

    ledcWrite(VIBRO_CHANNEL, duty);
}

// Temporarily silence vibro PWM before slow ADC reads to reduce interference.
void Control::pauseVibroForAdc()
{
    ledcWrite(VIBRO_CHANNEL, 0);
    delayMicroseconds(900);
}

// Restore the last commanded vibro PWM after ADC reads finish.
void Control::restoreVibroAfterAdc()
{
    ledcWrite(VIBRO_CHANNEL, vibroDuty);
}

// Read the shared FSR sensor and refresh force-feedback telemetry/output.
void Control::updateFsrInput(uint32_t nowMs)
{
    (void)nowMs;

    int fsrAdcRaw = readAdcAvgStable(current.fsrPin, FSR_SAMPLES);

    float alpha = (fsrAdcRaw > fsrAdcFiltered) ? FSR_PRESS_ALPHA : FSR_RELEASE_ALPHA;
    fsrAdcFiltered = smooth(fsrAdcFiltered, (float)fsrAdcRaw, alpha);

    fsrRaw = resistanceFromAdc((float)fsrAdcRaw, current.fsrPullupOhm);
    fsrFiltered = resistanceFromAdc(fsrAdcFiltered, current.fsrPullupOhm);

    tele.fsrRawOhm = sanitizeResistanceForTelemetry(fsrRaw);
    tele.fsrFilteredOhm = sanitizeResistanceForTelemetry(fsrFiltered);

    float forceN = fsrToNewton(fsrFiltered);
    tele.fsrForceN = forceN;

    bool pulseMode = false;
    uint8_t duty = computeVibroDuty(forceN, pulseMode);
    updateVibro(duty);
}

// Service exactly one slow analog source per loop so EMG stays on the hot path.
void Control::processOneSlowInput(uint32_t nowMs)
{
    const uint8_t totalSlots = NUM_PAIRS + 1;

    for (uint8_t step = 0; step < totalSlots; ++step)
    {
        const uint8_t slot = slowSensorCursor;
        slowSensorCursor++;
        if (slowSensorCursor >= totalSlots)
            slowSensorCursor = 0;

        if (slot < NUM_PAIRS)
        {
            const int idx = (int)slot;
            // Flex inputs are scheduled per pair because each pair can be flex-driven.
            if (current.pairInput[idx].inputSource != InputSource::Flex)
                continue;
            if (!isValidPin(current.servo[idx].servoPin))
                continue;
            if (!isValidPin(current.flex[idx].flexPin))
                continue;
            if (lastFlexSensorUpdateMs[idx] != 0 &&
                nowMs - lastFlexSensorUpdateMs[idx] < FLEX_SENSOR_PERIOD_MS)
            {
                continue;
            }

            pauseVibroForAdc();
            updateFlexInput(idx);
            restoreVibroAfterAdc();
            lastFlexSensorUpdateMs[idx] = nowMs;
            return;
        }

        // FSR is global, so it occupies the final slot in the round-robin schedule.
        if (!isValidPin(current.fsrPin))
            continue;
        if (lastFsrSensorUpdateMs != 0 &&
            nowMs - lastFsrSensorUpdateMs < FSR_SENSOR_PERIOD_MS)
        {
            continue;
        }

        pauseVibroForAdc();
        updateFsrInput(nowMs);
        lastFsrSensorUpdateMs = nowMs;
        return;
    }
}

// Reinitialize ADC, PWM, servo state, and EMG runtime after settings changes.
void Control::setupHardware()
{
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    if (isValidPin(current.fsrPin))
        analogSetPinAttenuation(current.fsrPin, ADC_11db);

    for (int i = 0; i < NUM_PAIRS; ++i)
    {
        if (isValidPin(current.flex[i].flexPin))
            analogSetPinAttenuation(current.flex[i].flexPin, ADC_11db);

        // Preconfigure ADC attenuation for the single EMG input pin when present.
        const EmgSettings &emgCfg = current.emg[i];
        if (isValidPin(emgCfg.pin))
            analogSetPinAttenuation(emgCfg.pin, ADC_11db);
    }

    pinMode(current.vibroPin, OUTPUT);
    ledcSetup(VIBRO_CHANNEL, current.vibroFreqHz, 8);
    ledcAttachPin(current.vibroPin, VIBRO_CHANNEL);
    ledcWrite(VIBRO_CHANNEL, 0);

    lastAdcPin = 0xFF;

    uint32_t now = millis();
    for (int i = 0; i < NUM_PAIRS; ++i)
    {
        if (servos[i].attached())
            servos[i].detach();
        servoAngleDeg[i] = current.servo[i].servoMaxDeg;
        targetAngleDeg[i] = servoAngleDeg[i];
        lastServoUpdate[i] = now;

        flexFiltered[i] = 0.0f;
        flexRaw[i] = 0.0f;

        flexStableInit[i] = false;
        flexStableOhm[i] = 0.0f;

        flexHistCount[i] = 0;
        flexHistPos[i] = 0;
        flexOutlierStrikes[i] = 0;
        for (int k = 0; k < FLEX_HISTORY; ++k)
            flexHist[i][k] = 0.0f;

        liveActive[i] = false;
        liveUntilMs[i] = 0;
        liveDeg[i] = 0.0f;
    }

    fsrFiltered = 0.0f;
    fsrRaw = 0.0f;
    fsrAdcFiltered = 0.0f;
    vibroDuty = 0;
    lastFsrSensorUpdateMs = 0;
    slowSensorCursor = 0;
    for (int i = 0; i < NUM_PAIRS; ++i)
        lastFlexSensorUpdateMs[i] = 0;

    emg.setAdcReader(&Control::adcReadBridge, this);
    emg.reset(current, tele);
}
