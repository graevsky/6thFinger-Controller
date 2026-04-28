// EMG control engine.
#pragma once

#include <Arduino.h>

#include "../config/settings.h"
#include "../config/telemetry.h"
#include "emg_models.h"

class EmgEngine
{
public:
    // ADC callback signature injected by the control subsystem.
    using AdcReadFn = int (*)(void *ctx, uint8_t pin, int samples);

    EmgEngine() = default;

    // Provide the ADC reader implementation used for EMG sampling.
    void setAdcReader(AdcReadFn fn, void *ctx);
    // Reset all per-pair EMG runtime state after configuration changes.
    void reset(const Settings &s, ControlTelemetry &tele);

    // Validate whether a pair has a usable EMG input pin.
    bool isPairConfigured(const Settings &s, int idx) const;
    // Mark EMG telemetry as inactive for pairs that are not using EMG.
    void setTelemetryInactive(ControlTelemetry &tele, int idx) const;

    // Run one EMG update step for a pair and refresh its telemetry.
    void updatePair(int idx, const Settings &s, uint32_t nowMs, ControlTelemetry &tele);
    // Convert the pair's current logical finger state into a servo angle.
    float targetAngleForPair(const Settings &s, int idx) const;

private:
    // Internal motion labels used only by the optional EMG mock interpreter.
    enum class MockMotion : uint8_t
    {
        Idle = 0,
        Snapshot = 1,
        Other = 2
    };

    // Logical servo state for bend/unfold toggling.
    enum class FingerState : uint8_t
    {
        Extended = 0,
        Bent = 1
    };

    // Normalized labels produced by the realtime binary classifier.
    enum class RealtimePrediction : uint8_t
    {
        None = 0,
        Other = 1,
        Movement = 2
    };

    // EMG runtime constants shared across all pairs.
    static constexpr int EMG_REAL_SAMPLES = 1;
    static constexpr float ACTIVE_RAW_THRESHOLD = 1850.0f;
    static constexpr uint8_t SNAPSHOT_END_OTHER_WINDOWS = 2;
    static constexpr bool EMG_SERIAL_DEBUG = true;

    // External ADC sampling hook.
    AdcReadFn adcReadFn = nullptr;
    void *adcReadCtx = nullptr;

    // Heuristic interpreter window state used by the optional EMG mock.
    bool interpActive[NUM_PAIRS] = {};
    uint32_t interpStartedMs[NUM_PAIRS] = {};
    uint32_t interpLastActiveMs[NUM_PAIRS] = {};
    float interpPeak[NUM_PAIRS] = {};
    uint16_t interpSamples[NUM_PAIRS] = {};

    // High-level gesture state and cooldown bookkeeping.
    FingerState fingerState[NUM_PAIRS] = {};
    uint32_t cooldownUntilMs[NUM_PAIRS] = {};
    uint8_t bendCounter[NUM_PAIRS] = {};
    uint8_t unfoldCounter[NUM_PAIRS] = {};
    bool targetBent[NUM_PAIRS] = {};
    uint32_t lastBendCountMs[NUM_PAIRS] = {};
    uint32_t lastUnfoldCountMs[NUM_PAIRS] = {};

    // Telemetry hold timers, realtime model runtime, and latest raw sample.
    uint32_t teleEventHoldUntilMs[NUM_PAIRS] = {};
    uint32_t teleActionHoldUntilMs[NUM_PAIRS] = {};
    uint32_t nextSampleUs[NUM_PAIRS] = {};
    EmgRealtimeModelState realtimeModelState[NUM_PAIRS] = {};
    float lastRawSample[NUM_PAIRS] = {};

    // Snapshot builder state for the realtime binary model.
    bool snapshotOpen[NUM_PAIRS] = {};
    uint8_t snapshotMovementCount[NUM_PAIRS] = {};
    uint8_t snapshotOtherStreak[NUM_PAIRS] = {};

private:
    // Small internal helpers used by setup and sampling.
    bool isValidPin(uint8_t pin) const;
    void resetPairState(int idx);
    void resetSnapshotBurst(int idx);

    // Sampling/model helpers.
    bool shouldUseMock(const Settings &s, int idx) const;
    void initRealtimeState(int idx);
    void makeMockSnapshot(uint32_t nowMs, float &outRaw) const;
    void readChannel(int idx, const Settings &s, uint32_t nowMs, float &outRaw) const;

    // Gesture classification helpers for mock and realtime paths.
    MockMotion classifyWindow(int idx) const;
    EmgEvent motionToEvent(MockMotion motion) const;
    EmgEvent processInterpreter(int idx, float raw, uint32_t nowMs);
    RealtimePrediction classifyRealtimeLabel(const char *label) const;
    EmgEvent finalizeSnapshotBurst(int idx, const EmgSettings &cfg);
    EmgEvent processRealtimeClassifier(int idx,
                                       const EmgSettings &cfg,
                                       float raw,
                                       RealtimePrediction *predictionOut,
                                       bool *snapshotStartedOut,
                                       bool *snapshotFinishedOut);

    // Telemetry and state-transition helpers.
    void setTeleEvent(ControlTelemetry &tele, int idx, EmgEvent event, uint32_t holdMs = 550);
    void setTeleAction(ControlTelemetry &tele, int idx, EmgAction action, uint32_t holdMs = 450);
    void refreshTeleTimers(ControlTelemetry &tele, int idx, uint32_t nowMs);
    void refreshSnapshotTimeout(int idx, const EmgSettings &cfg, uint32_t nowMs);
    uint8_t snapshotCounterForNextAction(int idx) const;
    uint8_t requiredSnapshotsForNextAction(int idx, const EmgSettings &cfg) const;
    void logRealtimeDecision(int idx,
                             const EmgSettings &cfg,
                             RealtimePrediction prediction,
                             bool snapshotStarted,
                             bool snapshotFinished) const;

    void applyEvent(int idx, const Settings &s, EmgEvent event, uint32_t nowMs, ControlTelemetry &tele);
};
