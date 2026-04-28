// Heuristic interpreter used by the EMG mock.
#include "emg_engine.h"

// Classify the collected active EMG window using simple peak and duration rules.
EmgEngine::MockMotion EmgEngine::classifyWindow(int idx) const
{
    const float samples = (float)interpSamples[idx];
    if (samples < 1.0f)
        return MockMotion::Idle;

    const float durationMs =
        (float)(interpLastActiveMs[idx] - interpStartedMs[idx] + 1U);
    const float peak = interpPeak[idx];

    if (peak < 1650.0f)
        return MockMotion::Idle;

    if (peak < 2500.0f || durationMs >= 270.0f)
        return MockMotion::Other;

    return MockMotion::Snapshot;
}

// Translate a mock motion label into the EMG event enum used elsewhere.
EmgEvent EmgEngine::motionToEvent(MockMotion motion) const
{
    switch (motion)
    {
    case MockMotion::Snapshot:
        return EmgEvent::Snapshot;

    case MockMotion::Other:
    case MockMotion::Idle:
    default:
        return EmgEvent::None;
    }
}

// Build an active window from mock samples and emit a snapshot event when motion ends.
EmgEvent EmgEngine::processInterpreter(int idx, float raw, uint32_t nowMs)
{
    const bool activeNow = isfinite(raw) && raw >= ACTIVE_RAW_THRESHOLD;
    const bool wasActive = interpActive[idx];

    // The interpreter starts a gesture window once the amplitude rises above a
    // simple threshold, then stores the strongest sample until activity stops.
    if (activeNow)
    {
        if (!wasActive)
        {
            interpActive[idx] = true;
            interpStartedMs[idx] = nowMs;
            interpLastActiveMs[idx] = nowMs;
            interpPeak[idx] = 0.0f;
            interpSamples[idx] = 0;
        }

        interpLastActiveMs[idx] = nowMs;
        if (interpSamples[idx] < 65530)
            interpSamples[idx]++;

        if (raw > interpPeak[idx])
            interpPeak[idx] = raw;

        return EmgEvent::None;
    }

    if (!wasActive)
        return EmgEvent::None;

    interpActive[idx] = false;

    const float activeDuration =
        (float)(interpLastActiveMs[idx] - interpStartedMs[idx] + 1U);
    if (activeDuration < 80.0f)
    {
        interpSamples[idx] = 0;
        return EmgEvent::None;
    }

    const MockMotion motion = classifyWindow(idx);
    interpSamples[idx] = 0;
    return motionToEvent(motion);
}
