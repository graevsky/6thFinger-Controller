// EMG state machine transitions, telemetry hold timers, and per-pair updates.
#include "emg_engine.h"

#include <math.h>

// Hold a low-level EMG event in telemetry long enough for BLE to observe it.
void EmgEngine::setTeleEvent(ControlTelemetry &tele,
                             int idx,
                             EmgEvent event,
                             uint32_t holdMs)
{
    tele.emgEvent[idx] = (int8_t)event;
    teleEventHoldUntilMs[idx] = millis() + holdMs;
}

// Hold a high-level EMG action in telemetry long enough for BLE to observe it.
void EmgEngine::setTeleAction(ControlTelemetry &tele,
                              int idx,
                              EmgAction action,
                              uint32_t holdMs)
{
    tele.emgAction[idx] = (int8_t)action;
    teleActionHoldUntilMs[idx] = millis() + holdMs;
}

// Expire held telemetry fields once their display timeout elapses.
void EmgEngine::refreshTeleTimers(ControlTelemetry &tele, int idx, uint32_t nowMs)
{
    if (teleEventHoldUntilMs[idx] != 0 && nowMs >= teleEventHoldUntilMs[idx])
    {
        tele.emgEvent[idx] = (int8_t)EmgEvent::None;
        teleEventHoldUntilMs[idx] = 0;
    }

    if (teleActionHoldUntilMs[idx] != 0 && nowMs >= teleActionHoldUntilMs[idx])
    {
        tele.emgAction[idx] = (int8_t)EmgAction::None;
        teleActionHoldUntilMs[idx] = 0;
    }
}

// Reset bend/unfold snapshot counters when the configured timeout is exceeded.
void EmgEngine::refreshSnapshotTimeout(int idx,
                                       const EmgSettings &cfg,
                                       uint32_t nowMs)
{
    const uint32_t timeoutMs = (uint32_t)cfg.snapshotTimeoutSec * 1000UL;

    if (fingerState[idx] == FingerState::Extended)
    {
        if (bendCounter[idx] > 0 && lastBendCountMs[idx] != 0 &&
            nowMs - lastBendCountMs[idx] >= timeoutMs)
        {
            bendCounter[idx] = 0;
            lastBendCountMs[idx] = 0;
        }
        return;
    }

    if (unfoldCounter[idx] > 0 && lastUnfoldCountMs[idx] != 0 &&
        nowMs - lastUnfoldCountMs[idx] >= timeoutMs)
    {
        unfoldCounter[idx] = 0;
        lastUnfoldCountMs[idx] = 0;
    }
}

// Convert the logical bent/extended state into the correct servo endpoint.
float EmgEngine::targetAngleForPair(const Settings &s, int idx) const
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return 0.0f;

    const ServoSettings &servoCfg = s.servo[idx];
    const EmgSettings &emgCfg = s.emg[idx];

    const float bendAngle =
        emgCfg.reverseDirection ? servoCfg.servoMaxDeg : servoCfg.servoMinDeg;
    const float unfoldAngle =
        emgCfg.reverseDirection ? servoCfg.servoMinDeg : servoCfg.servoMaxDeg;

    return targetBent[idx] ? bendAngle : unfoldAngle;
}

// Return the current logical snapshot progress toward the next allowed action.
uint8_t EmgEngine::snapshotCounterForNextAction(int idx) const
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return 0;

    return (fingerState[idx] == FingerState::Extended) ? bendCounter[idx]
                                                       : unfoldCounter[idx];
}

// Return how many snapshots are required for the next bend or unfold action.
uint8_t EmgEngine::requiredSnapshotsForNextAction(int idx,
                                                  const EmgSettings &cfg) const
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return 0;

    return (fingerState[idx] == FingerState::Extended) ? cfg.bendSnapshotsToBend
                                                       : cfg.bendSnapshotsToUnfold;
}

// Emit one concise debug line for every ready realtime classifier label.
void EmgEngine::logRealtimeDecision(int idx,
                                    const EmgSettings &cfg,
                                    RealtimePrediction prediction,
                                    bool snapshotStarted,
                                    bool snapshotFinished) const
{
    if (!EMG_SERIAL_DEBUG || prediction == RealtimePrediction::None)
        return;

    const char *label = "unknown";
    if (prediction == RealtimePrediction::Other)
        label = "other";
    else if (prediction == RealtimePrediction::Movement)
        label = "movement";

    // Serial.print("pair: ");
    // Serial.print(idx);
    // Serial.print(" | action: ");
    // Serial.print(label);
    // Serial.print(" | snapshot started: ");
    // Serial.print(snapshotStarted ? "yes" : "no");
    // Serial.print(" | snapshot finished: ");
    // Serial.print(snapshotFinished ? "yes" : "no");
    // Serial.print(" | snapshot counter: ");
    // Serial.print((int)snapshotCounterForNextAction(idx));
    // Serial.print(" | required snapshots for next action: ");
    // Serial.println((int)requiredSnapshotsForNextAction(idx, cfg));
}

// Apply one snapshot event to the per-pair state machine and mirror it into telemetry.
void EmgEngine::applyEvent(int idx,
                           const Settings &s,
                           EmgEvent event,
                           uint32_t nowMs,
                           ControlTelemetry &tele)
{
    const EmgSettings &cfg = s.emg[idx];

    tele.emgBendProgress[idx] = (int8_t)bendCounter[idx];
    tele.emgUnfoldProgress[idx] = (int8_t)unfoldCounter[idx];
    tele.emgCooldownMs[idx] =
        (nowMs < cooldownUntilMs[idx]) ? (int16_t)(cooldownUntilMs[idx] - nowMs)
                                       : 0;

    if (event != EmgEvent::Snapshot)
        return;

    setTeleEvent(tele, idx, event);

    if (fingerState[idx] == FingerState::Extended)
    {
        if (bendCounter[idx] < cfg.bendSnapshotsToBend)
            bendCounter[idx]++;

        lastBendCountMs[idx] = nowMs;
        tele.emgBendProgress[idx] = (int8_t)bendCounter[idx];

        if (bendCounter[idx] >= cfg.bendSnapshotsToBend)
        {
            fingerState[idx] = FingerState::Bent;
            targetBent[idx] = true;
            bendCounter[idx] = 0;
            unfoldCounter[idx] = 0;
            lastBendCountMs[idx] = 0;
            lastUnfoldCountMs[idx] = 0;
            cooldownUntilMs[idx] =
                nowMs + (uint32_t)cfg.minUnfoldDelaySec * 1000UL;
            tele.emgBendProgress[idx] = 0;
            tele.emgUnfoldProgress[idx] = 0;
            tele.emgCooldownMs[idx] =
                (int16_t)(cooldownUntilMs[idx] - nowMs);
            setTeleAction(tele, idx, EmgAction::Bend);
        }
        return;
    }

    if (nowMs < cooldownUntilMs[idx])
    {
        tele.emgCooldownMs[idx] = (int16_t)(cooldownUntilMs[idx] - nowMs);
        setTeleAction(tele, idx, EmgAction::CooldownIgnored);
        return;
    }

    tele.emgCooldownMs[idx] = 0;

    if (unfoldCounter[idx] < cfg.bendSnapshotsToUnfold)
        unfoldCounter[idx]++;

    lastUnfoldCountMs[idx] = nowMs;
    tele.emgUnfoldProgress[idx] = (int8_t)unfoldCounter[idx];

    if (unfoldCounter[idx] >= cfg.bendSnapshotsToUnfold)
    {
        fingerState[idx] = FingerState::Extended;
        targetBent[idx] = false;
        bendCounter[idx] = 0;
        unfoldCounter[idx] = 0;
        lastBendCountMs[idx] = 0;
        lastUnfoldCountMs[idx] = 0;
        tele.emgBendProgress[idx] = 0;
        tele.emgUnfoldProgress[idx] = 0;
        setTeleAction(tele, idx, EmgAction::Unfold);
    }
}

// Run one pair update: sample at the configured cadence, classify, and update state.
void EmgEngine::updatePair(int idx,
                           const Settings &s,
                           uint32_t nowMs,
                           ControlTelemetry &tele)
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return;

    refreshTeleTimers(tele, idx, nowMs);

    const EmgSettings &cfg = s.emg[idx];
    refreshSnapshotTimeout(idx, cfg, nowMs);

    tele.emgSource[idx] = (int8_t)InputSource::Emg;
    tele.emgBendProgress[idx] = (int8_t)bendCounter[idx];
    tele.emgUnfoldProgress[idx] = (int8_t)unfoldCounter[idx];
    tele.emgCooldownMs[idx] =
        (nowMs < cooldownUntilMs[idx]) ? (int16_t)(cooldownUntilMs[idx] - nowMs)
                                       : 0;
    tele.emgCh0[idx] = lastRawSample[idx];

    if (!isPairConfigured(s, idx))
    {
        tele.emgCh0[idx] = NAN;
        return;
    }

    // The mock path uses a tiny heuristic interpreter because generated models
    // are reserved for real ADC data.
    if (shouldUseMock(s, idx))
    {
        float raw = NAN;
        readChannel(idx, s, nowMs, raw);

        lastRawSample[idx] = raw;
        tele.emgCh0[idx] = raw;

        EmgEvent event = processInterpreter(idx, raw, nowMs);
        applyEvent(idx, s, event, nowMs, tele);
        return;
    }

    // The realtime path keeps its own sampling cadence even if update() is called faster.
    const uint32_t nowUs = micros();
    if (nextSampleUs[idx] == 0)
        nextSampleUs[idx] = nowUs;

    if ((int32_t)(nowUs - nextSampleUs[idx]) < 0)
        return;

    nextSampleUs[idx] += EMG_REALTIME_SAMPLE_PERIOD_US;

    float raw = NAN;
    readChannel(idx, s, nowMs, raw);

    lastRawSample[idx] = raw;
    tele.emgCh0[idx] = raw;

    if (isfinite(raw))
    {
        RealtimePrediction prediction = RealtimePrediction::None;
        bool snapshotStarted = false;
        bool snapshotFinished = false;
        EmgEvent event = processRealtimeClassifier(idx,
                                                   cfg,
                                                   raw,
                                                   &prediction,
                                                   &snapshotStarted,
                                                   &snapshotFinished);
        applyEvent(idx, s, event, nowMs, tele);
        logRealtimeDecision(idx, cfg, prediction, snapshotStarted, snapshotFinished);
    }

    const uint32_t afterReadUs = micros();
    if ((int32_t)(afterReadUs - nextSampleUs[idx]) >
        (int32_t)(5U * EMG_REALTIME_SAMPLE_PERIOD_US))
    {
        nextSampleUs[idx] = afterReadUs + EMG_REALTIME_SAMPLE_PERIOD_US;
    }
}
