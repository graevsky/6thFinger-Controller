// EMG engine core helpers: mock generation, raw sampling, and shared resets.
#include "emg_engine.h"

#include <math.h>
#include <string.h>

// ================= EMG ADC MOCK =================
// Development-only EMG mock. It is compiled out by default and emulates the
// single-channel binary model input used by the production firmware.
#define ENABLE_EMG_ADC_MOCK 0

#if ENABLE_EMG_ADC_MOCK
static constexpr uint8_t MOCK_EMG_PIN = 34;
#endif
// ================= /EMG ADC MOCK =================

// Install the ADC callback provided by the control subsystem.
void EmgEngine::setAdcReader(AdcReadFn fn, void *ctx)
{
    adcReadFn = fn;
    adcReadCtx = ctx;
}

// Treat 0 and UNUSED_PIN as disconnected pins.
bool EmgEngine::isValidPin(uint8_t pin) const
{
    return pin != 0 && pin != UNUSED_PIN;
}

// Drop any in-progress realtime snapshot burst for one pair.
void EmgEngine::resetSnapshotBurst(int idx)
{
    snapshotOpen[idx] = false;
    snapshotMovementCount[idx] = 0;
    snapshotOtherStreak[idx] = 0;
}

// Reset every runtime buffer and state machine field for one EMG pair.
void EmgEngine::resetPairState(int idx)
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return;

    interpActive[idx] = false;
    interpStartedMs[idx] = 0;
    interpLastActiveMs[idx] = 0;
    interpPeak[idx] = 0.0f;
    interpSamples[idx] = 0;

    fingerState[idx] = FingerState::Extended;
    cooldownUntilMs[idx] = 0;
    bendCounter[idx] = 0;
    unfoldCounter[idx] = 0;
    targetBent[idx] = false;
    lastBendCountMs[idx] = 0;
    lastUnfoldCountMs[idx] = 0;

    teleEventHoldUntilMs[idx] = 0;
    teleActionHoldUntilMs[idx] = 0;
    nextSampleUs[idx] = 0;
    lastRawSample[idx] = NAN;

    resetSnapshotBurst(idx);
    memset(&realtimeModelState[idx], 0, sizeof(realtimeModelState[idx]));
}

// Mark EMG telemetry as unused for pairs that are inactive or flex-driven.
void EmgEngine::setTelemetryInactive(ControlTelemetry &tele, int idx) const
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return;

    tele.emgSource[idx] = -1;
    tele.emgEvent[idx] = (int8_t)EmgEvent::None;
    tele.emgAction[idx] = (int8_t)EmgAction::None;
    tele.emgCooldownMs[idx] = -1;
    tele.emgBendProgress[idx] = -1;
    tele.emgUnfoldProgress[idx] = -1;
    tele.emgCh0[idx] = NAN;
}

// Reset all pairs after boot or settings reconfiguration.
void EmgEngine::reset(const Settings &s, ControlTelemetry &tele)
{
    for (int i = 0; i < NUM_PAIRS; ++i)
    {
        resetPairState(i);
        if (s.emg[i].activePinsValid())
            initRealtimeState(i);
        setTelemetryInactive(tele, i);
    }
}

// Check whether the current EMG pin is connected.
bool EmgEngine::isPairConfigured(const Settings &s, int idx) const
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return false;
    return s.emg[idx].activePinsValid();
}

// Decide whether this pair should use the built-in EMG mock instead of ADC.
bool EmgEngine::shouldUseMock(const Settings &s, int idx) const
{
    if (!isPairConfigured(s, idx))
        return false;

#if !ENABLE_EMG_ADC_MOCK
    (void)s;
    (void)idx;
    return false;
#else
    return s.emg[idx].pin == MOCK_EMG_PIN;
#endif
}

// Reset the generated model state for one pair.
void EmgEngine::initRealtimeState(int idx)
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return;

    emgRealtimeInit(&realtimeModelState[idx]);
    nextSampleUs[idx] = 0;
    lastRawSample[idx] = NAN;
    resetSnapshotBurst(idx);
}

// Generate one mock EMG frame that alternates between snapshot-like and
// other-like activity so the state machine can be exercised without electrodes.
void EmgEngine::makeMockSnapshot(uint32_t nowMs, float &outRaw) const
{
    static const uint16_t BASELINE = 1220;
    static const uint16_t MID_A = 980;
    static const uint16_t HIGH_A = 2100;

    struct MockStep
    {
        MockMotion motion;
        uint16_t durationMs;
    };

    static const MockStep sequence[] = {
        {MockMotion::Idle, 850},
        {MockMotion::Snapshot, 230},
        {MockMotion::Idle, 200},
        {MockMotion::Other, 320},
        {MockMotion::Idle, 180},
        {MockMotion::Other, 340},
        {MockMotion::Idle, 180},
        {MockMotion::Snapshot, 240},
        {MockMotion::Idle, 900},
    };

    uint32_t period = 0;
    for (size_t i = 0; i < sizeof(sequence) / sizeof(sequence[0]); ++i)
        period += sequence[i].durationMs;

    uint32_t t = (period > 0) ? (nowMs % period) : 0;
    MockMotion motion = MockMotion::Idle;
    uint16_t phaseMs = 0;
    uint16_t phaseDur = 1;

    for (size_t i = 0; i < sizeof(sequence) / sizeof(sequence[0]); ++i)
    {
        if (t < sequence[i].durationMs)
        {
            motion = sequence[i].motion;
            phaseMs = (uint16_t)t;
            phaseDur = sequence[i].durationMs;
            break;
        }
        t -= sequence[i].durationMs;
    }

    float env = 0.0f;
    if (motion != MockMotion::Idle)
    {
        const float k = (float)phaseMs / (float)phaseDur;
        if (motion == MockMotion::Snapshot)
        {
            if (k < 0.25f)
                env = k / 0.25f;
            else if (k < 0.70f)
                env = 1.0f;
            else
                env = 1.0f - (k - 0.70f) / 0.30f;
        }
        else
        {
            if (k < 0.20f)
                env = k / 0.20f;
            else if (k < 0.80f)
                env = 0.72f;
            else
                env = 0.72f * (1.0f - (k - 0.80f) / 0.20f);
        }

        if (env < 0.0f)
            env = 0.0f;
        if (env > 1.0f)
            env = 1.0f;
    }

    outRaw = (float)BASELINE;
    if (motion == MockMotion::Snapshot)
        outRaw = BASELINE + HIGH_A * env;
    else if (motion == MockMotion::Other)
        outRaw = BASELINE + MID_A * env;

    const int wobble = ((int)(nowMs / 17U) % 25) - 12;
    outRaw += (float)wobble;
    if (outRaw < 0.0f)
        outRaw = 0.0f;
    if (outRaw > 4095.0f)
        outRaw = 4095.0f;
}

// Read the single active EMG channel for a pair, either from mock data or ADC.
void EmgEngine::readChannel(int idx,
                            const Settings &s,
                            uint32_t nowMs,
                            float &outRaw) const
{
    outRaw = NAN;

    if (!isPairConfigured(s, idx))
        return;

    if (shouldUseMock(s, idx))
    {
        makeMockSnapshot(nowMs, outRaw);
        return;
    }

    if (adcReadFn == nullptr)
        return;

    const uint8_t pin = s.emg[idx].pin;
    if (!isValidPin(pin))
        return;

    outRaw = (float)adcReadFn(adcReadCtx, pin, EMG_REAL_SAMPLES);
}
