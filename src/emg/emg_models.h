// Thin wrapper around the single-channel binary EMG model.
#pragma once

#include <Arduino.h>
#include <string.h>

#include "emg_features.h"
#include "realtime_1ch_binary_model.h"

// Runtime state for one instance of the 1-channel binary classifier.
typedef struct EmgRealtimeModelState
{
    emg_rt_filter_bank_t filter_bank;
    float ring[EMG_RT1B_WINDOW_SAMPLES * EMG_RT1B_NUM_CHANNELS];
    float ordered_window[EMG_RT1B_WINDOW_SAMPLES * EMG_RT1B_NUM_CHANNELS];
    float scratch[EMG_RT1B_FEATURE_SCRATCH_SIZE];
    float features[EMG_RT1B_NUM_FEATURES];
    size_t write_index;
    size_t count;
    size_t hop_cursor;
} EmgRealtimeModelState;

static constexpr uint32_t EMG_REALTIME_SAMPLE_PERIOD_US =
    1000000UL / (uint32_t)EMG_RT1B_FS_HZ;

// Reset the generated model state and its filter bank.
static inline void emgRealtimeInit(EmgRealtimeModelState *state)
{
    memset(state, 0, sizeof(*state));
    emg_rt1b_init_filter_bank(&state->filter_bank);
}

// Push one raw EMG sample and emit a label when the current hop is complete.
static inline bool emgRealtimePushFrame(EmgRealtimeModelState *state,
                                        float raw_sample,
                                        const char **label_out)
{
    const float filtered_sample =
        emg_rt_filter_bank_process_sample(&state->filter_bank, raw_sample);

    emg_rt_ring_push_frame(state->ring,
                           EMG_RT1B_WINDOW_SAMPLES,
                           EMG_RT1B_NUM_CHANNELS,
                           &state->write_index,
                           &state->count,
                           &filtered_sample);

    if (state->count < EMG_RT1B_WINDOW_SAMPLES)
        return false;

    if (state->hop_cursor + 1U < EMG_RT1B_HOP_SAMPLES)
    {
        state->hop_cursor += 1U;
        return false;
    }

    state->hop_cursor = 0U;

    emg_rt_ring_copy_ordered_frames(state->ring,
                                    EMG_RT1B_WINDOW_SAMPLES,
                                    EMG_RT1B_NUM_CHANNELS,
                                    state->write_index,
                                    state->count,
                                    state->ordered_window);

    emg_rt1b_extract_features(
        state->ordered_window, state->scratch, state->features);
    *label_out = emg_rt1b_predict_label(state->features);
    return true;
}
