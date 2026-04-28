// Realtime binary-model label normalization and snapshot detection logic.
#include "emg_engine.h"

#include <string.h>

// Normalize generated model labels into the runtime enum used by the EMG engine.
EmgEngine::RealtimePrediction EmgEngine::classifyRealtimeLabel(const char *label) const
{
    if (label == nullptr || label[0] == '\0')
        return RealtimePrediction::None;

    if (strcmp(label, "movement") == 0)
        return RealtimePrediction::Movement;
    if (strcmp(label, "other") == 0)
        return RealtimePrediction::Other;

    return RealtimePrediction::None;
}

// Close the current movement burst and emit one snapshot when it was large enough.
EmgEvent EmgEngine::finalizeSnapshotBurst(int idx, const EmgSettings &cfg)
{
    const bool snapshotReady =
        snapshotOpen[idx] && snapshotMovementCount[idx] >= cfg.snapshotSize;

    resetSnapshotBurst(idx);
    return snapshotReady ? EmgEvent::Snapshot : EmgEvent::None;
}

// Push one sample into the binary model and convert classifier labels into snapshots.
EmgEvent EmgEngine::processRealtimeClassifier(int idx,
                                              const EmgSettings &cfg,
                                              float raw,
                                              RealtimePrediction *predictionOut,
                                              bool *snapshotStartedOut,
                                              bool *snapshotFinishedOut)
{
    if (predictionOut != nullptr)
        *predictionOut = RealtimePrediction::None;
    if (snapshotStartedOut != nullptr)
        *snapshotStartedOut = false;
    if (snapshotFinishedOut != nullptr)
        *snapshotFinishedOut = false;

    const char *label = nullptr;
    const bool ready =
        emgRealtimePushFrame(&realtimeModelState[idx], raw, &label);
    if (!ready || label == nullptr)
        return EmgEvent::None;

    const RealtimePrediction pred = classifyRealtimeLabel(label);
    if (predictionOut != nullptr)
        *predictionOut = pred;

    if (pred == RealtimePrediction::Movement)
    {
        const bool wasOpen = snapshotOpen[idx];
        snapshotOpen[idx] = true;
        snapshotOtherStreak[idx] = 0;

        if (snapshotMovementCount[idx] < 255)
            snapshotMovementCount[idx]++;
        if (!wasOpen && snapshotStartedOut != nullptr)
            *snapshotStartedOut = true;
        return EmgEvent::None;
    }

    if (!snapshotOpen[idx])
        return EmgEvent::None;

    // A short "other" gap is allowed inside one snapshot, but a longer gap
    // closes the burst and decides whether the collected movement windows count.
    if (snapshotOtherStreak[idx] < 255)
        snapshotOtherStreak[idx]++;

    if (snapshotOtherStreak[idx] < SNAPSHOT_END_OTHER_WINDOWS)
        return EmgEvent::None;

    if (snapshotFinishedOut != nullptr)
        *snapshotFinishedOut = true;
    return finalizeSnapshotBurst(idx, cfg);
}
