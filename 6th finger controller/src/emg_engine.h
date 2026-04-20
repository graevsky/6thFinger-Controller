#pragma once
#include <Arduino.h>
#include "settings.h"
#include "telemetry.h"

class EmgEngine
{
public:
    using AdcReadFn = int (*)(void *ctx, uint8_t pin, int samples);

    EmgEngine() = default;

    void setAdcReader(AdcReadFn fn, void *ctx);
    void reset(const Settings &s, ControlTelemetry &tele);

    bool isPairConfigured(const Settings &s, int idx) const;
    void setTelemetryInactive(ControlTelemetry &tele, int idx) const;

    void updatePair(int idx, const Settings &s, uint32_t nowMs, ControlTelemetry &tele);
    float targetAngleForPair(const Settings &s, int idx) const;

private:
    enum class MockMotion : uint8_t
    {
        Idle = 0,
        FingerBend = 1,
        FingerUnfold = 2,
        Other = 3
    };

    enum class FingerState : uint8_t
    {
        Extended = 0,
        Bent = 1
    };

    static constexpr int EMG_SAMPLES = 4;
    static constexpr float ACTIVE_RAW_THRESHOLD = 1850.0f;

    AdcReadFn adcReadFn = nullptr;
    void *adcReadCtx = nullptr;

    FingerState fingerState[NUM_PAIRS] = {};
    bool interpActive[NUM_PAIRS] = {};
    uint32_t interpStartedMs[NUM_PAIRS] = {};
    uint32_t interpLastActiveMs[NUM_PAIRS] = {};
    float interpPeak[NUM_PAIRS][3] = {};
    uint16_t interpSamples[NUM_PAIRS] = {};

    uint32_t cooldownUntilMs[NUM_PAIRS] = {};
    uint8_t bendCounter[NUM_PAIRS] = {};
    uint8_t unfoldCounter[NUM_PAIRS] = {};
    bool targetBent[NUM_PAIRS] = {};

    uint32_t teleEventHoldUntilMs[NUM_PAIRS] = {};
    uint32_t teleActionHoldUntilMs[NUM_PAIRS] = {};

private:
    bool isValidPin(uint8_t pin) const;
    void resetPairState(int idx);

    bool shouldUseMock(const Settings &s, int idx) const;
    void makeMockSnapshot(const EmgSettings &cfg, uint32_t nowMs, float outRaw[3]) const;
    void readChannels(int idx, const Settings &s, uint32_t nowMs, float outRaw[3]) const;

    MockMotion classifyWindow(int idx, int channels) const;
    EmgEvent motionToEvent(EmgMode mode, MockMotion motion) const;
    EmgEvent processInterpreter(int idx, const Settings &s, const float raw[3], uint32_t nowMs);

    void setTeleEvent(ControlTelemetry &tele, int idx, EmgEvent event, uint32_t holdMs = 550);
    void setTeleAction(ControlTelemetry &tele, int idx, EmgAction action, uint32_t holdMs = 450);
    void refreshTeleTimers(ControlTelemetry &tele, int idx, uint32_t nowMs);

    void applyEvent(int idx, const Settings &s, EmgEvent event, uint32_t nowMs, ControlTelemetry &tele);
};
