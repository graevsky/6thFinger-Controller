/* Realtime EMG feature extraction helpers shared by all generated classifiers.
 * This file is generated/model-side infrastructure and is documented at the
 * file level to avoid cluttering the extracted feature math.
 */
#pragma once

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef EMG_RT_MAX_SOS_SECTIONS
#define EMG_RT_MAX_SOS_SECTIONS 8
#endif

#ifndef EMG_RT_MAX_AR_ORDER
#define EMG_RT_MAX_AR_ORDER 6
#endif

#ifndef EMG_RT_BASE_FEATURES_1CH
#define EMG_RT_BASE_FEATURES_1CH 12
#endif

typedef enum emg_rt_center_mode_t {
    EMG_RT_CENTER_MEAN = 0,
    EMG_RT_CENTER_MEDIAN = 1,
} emg_rt_center_mode_t;

typedef enum emg_rt_clip_normalize_mode_t {
    EMG_RT_CLIP_NORMALIZE_NONE = 0,
    EMG_RT_CLIP_NORMALIZE_STD = 1,
    EMG_RT_CLIP_NORMALIZE_ROBUST = 2,
} emg_rt_clip_normalize_mode_t;

typedef struct emg_rt_sos_coeff_t {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
} emg_rt_sos_coeff_t;

typedef struct emg_rt_sos_section_t {
    emg_rt_sos_coeff_t coeff;
    float z1;
    float z2;
} emg_rt_sos_section_t;

typedef struct emg_rt_filter_bank_t {
    size_t count;
    emg_rt_sos_section_t sections[EMG_RT_MAX_SOS_SECTIONS];
} emg_rt_filter_bank_t;

static inline size_t emg_rt_feature_count_1ch(int ar_order) {
    if (ar_order < 0) {
        ar_order = 0;
    }
    if (ar_order > EMG_RT_MAX_AR_ORDER) {
        ar_order = EMG_RT_MAX_AR_ORDER;
    }
    return (size_t)(EMG_RT_BASE_FEATURES_1CH + ar_order);
}

static inline size_t emg_rt_feature_count(size_t num_channels, int ar_order, int segment_bins) {
    if (num_channels == 0U) {
        return 0U;
    }
    if (segment_bins < 0) {
        segment_bins = 0;
    }
    size_t base = num_channels * emg_rt_feature_count_1ch(ar_order);
    size_t pairs = (num_channels * (num_channels - 1U)) / 2U;
    size_t cross = pairs * 7U;
    size_t segment = (size_t)segment_bins * num_channels * 3U;
    return base + cross + segment;
}

static inline void emg_rt_filter_bank_init(
    emg_rt_filter_bank_t *bank,
    const emg_rt_sos_coeff_t *coeffs,
    size_t count
) {
    if (count > EMG_RT_MAX_SOS_SECTIONS) {
        count = EMG_RT_MAX_SOS_SECTIONS;
    }
    bank->count = count;
    for (size_t idx = 0; idx < count; ++idx) {
        bank->sections[idx].coeff = coeffs[idx];
        bank->sections[idx].z1 = 0.0f;
        bank->sections[idx].z2 = 0.0f;
    }
}

static inline void emg_rt_filter_bank_reset(emg_rt_filter_bank_t *bank) {
    for (size_t idx = 0; idx < bank->count; ++idx) {
        bank->sections[idx].z1 = 0.0f;
        bank->sections[idx].z2 = 0.0f;
    }
}

static inline float emg_rt_sos_process_sample(emg_rt_sos_section_t *section, float x) {
    float y = section->coeff.b0 * x + section->z1;
    section->z1 = section->coeff.b1 * x - section->coeff.a1 * y + section->z2;
    section->z2 = section->coeff.b2 * x - section->coeff.a2 * y;
    return y;
}

static inline float emg_rt_filter_bank_process_sample(emg_rt_filter_bank_t *bank, float sample) {
    float y = sample;
    for (size_t idx = 0; idx < bank->count; ++idx) {
        y = emg_rt_sos_process_sample(&bank->sections[idx], y);
    }
    return y;
}

static inline void emg_rt_ring_push(
    float *buffer,
    size_t capacity,
    size_t *write_index,
    size_t *count,
    float sample
) {
    if (capacity == 0) {
        return;
    }
    buffer[*write_index] = sample;
    *write_index = (*write_index + 1U) % capacity;
    if (*count < capacity) {
        *count += 1U;
    }
}

static inline void emg_rt_ring_push_frame(
    float *buffer,
    size_t capacity_frames,
    size_t num_channels,
    size_t *write_index,
    size_t *count,
    const float *frame
) {
    if (capacity_frames == 0U || num_channels == 0U) {
        return;
    }
    size_t offset = (*write_index) * num_channels;
    for (size_t channel_idx = 0; channel_idx < num_channels; ++channel_idx) {
        buffer[offset + channel_idx] = frame[channel_idx];
    }
    *write_index = (*write_index + 1U) % capacity_frames;
    if (*count < capacity_frames) {
        *count += 1U;
    }
}

static inline void emg_rt_ring_copy_ordered(
    const float *buffer,
    size_t capacity,
    size_t write_index,
    size_t count,
    float *out
) {
    if (count == 0 || capacity == 0) {
        return;
    }
    size_t start = (count < capacity) ? 0U : write_index;
    for (size_t idx = 0; idx < count; ++idx) {
        out[idx] = buffer[(start + idx) % capacity];
    }
}

static inline void emg_rt_ring_copy_ordered_frames(
    const float *buffer,
    size_t capacity_frames,
    size_t num_channels,
    size_t write_index,
    size_t count,
    float *out
) {
    if (count == 0U || capacity_frames == 0U || num_channels == 0U) {
        return;
    }
    size_t start = (count < capacity_frames) ? 0U : write_index;
    for (size_t frame_idx = 0; frame_idx < count; ++frame_idx) {
        size_t source_frame = (start + frame_idx) % capacity_frames;
        for (size_t channel_idx = 0; channel_idx < num_channels; ++channel_idx) {
            out[frame_idx * num_channels + channel_idx] = buffer[source_frame * num_channels + channel_idx];
        }
    }
}

static inline void emg_rt_insertion_sort(float *x, size_t n) {
    for (size_t i = 1; i < n; ++i) {
        float key = x[i];
        size_t j = i;
        while (j > 0 && x[j - 1] > key) {
            x[j] = x[j - 1];
            --j;
        }
        x[j] = key;
    }
}

static inline float emg_rt_center_value(
    const float *input,
    size_t n,
    emg_rt_center_mode_t center_mode,
    float *scratch
) {
    if (center_mode == EMG_RT_CENTER_MEAN) {
        float sum = 0.0f;
        for (size_t idx = 0; idx < n; ++idx) {
            sum += input[idx];
        }
        return sum / (float)n;
    }

    memcpy(scratch, input, n * sizeof(float));
    emg_rt_insertion_sort(scratch, n);
    if ((n & 1U) == 0U) {
        size_t right = n / 2U;
        return 0.5f * (scratch[right - 1U] + scratch[right]);
    }
    return scratch[n / 2U];
}

static inline void emg_rt_autocorr(
    const float *x,
    size_t n,
    int order,
    float *out
) {
    for (int lag = 0; lag <= order; ++lag) {
        float acc = 0.0f;
        for (size_t idx = 0; idx + (size_t)lag < n; ++idx) {
            acc += x[idx] * x[idx + (size_t)lag];
        }
        out[lag] = acc / (float)n;
    }
}

static inline void emg_rt_ar_yule_walker(
    const float *x,
    size_t n,
    int order,
    float *out
) {
    if (order <= 0) {
        return;
    }
    if (order > EMG_RT_MAX_AR_ORDER) {
        order = EMG_RT_MAX_AR_ORDER;
    }

    float autocorr[EMG_RT_MAX_AR_ORDER + 1] = {0};
    float a[EMG_RT_MAX_AR_ORDER] = {0};
    float next[EMG_RT_MAX_AR_ORDER] = {0};
    emg_rt_autocorr(x, n, order, autocorr);

    if (fabsf(autocorr[0]) < 1e-12f) {
        for (int idx = 0; idx < order; ++idx) {
            out[idx] = 0.0f;
        }
        return;
    }

    float error = autocorr[0];
    for (int i = 0; i < order; ++i) {
        float acc = autocorr[i + 1];
        for (int j = 0; j < i; ++j) {
            acc -= a[j] * autocorr[i - j];
        }
        float reflection = acc / (error + 1e-8f);
        next[i] = reflection;
        for (int j = 0; j < i; ++j) {
            next[j] = a[j] - reflection * a[i - 1 - j];
        }
        for (int j = 0; j <= i; ++j) {
            a[j] = next[j];
        }
        error *= (1.0f - reflection * reflection);
        if (error < 1e-8f) {
            error = 1e-8f;
        }
    }

    for (int idx = 0; idx < order; ++idx) {
        out[idx] = a[idx];
    }
}

static inline size_t emg_rt_extract_features_1ch(
    const float *input,
    size_t n,
    int ar_order,
    emg_rt_center_mode_t center_mode,
    float *scratch,
    float *out_features
) {
    if (n < 3U) {
        return 0U;
    }
    if (ar_order < 0) {
        ar_order = 0;
    }
    if (ar_order > EMG_RT_MAX_AR_ORDER) {
        ar_order = EMG_RT_MAX_AR_ORDER;
    }

    float center = emg_rt_center_value(input, n, center_mode, scratch);
    for (size_t idx = 0; idx < n; ++idx) {
        scratch[idx] = input[idx] - center;
    }

    float sum_abs = 0.0f;
    float sum_sq = 0.0f;
    float min_value = scratch[0];
    float max_value = scratch[0];
    float max_abs = fabsf(scratch[0]);
    for (size_t idx = 0; idx < n; ++idx) {
        float value = scratch[idx];
        float abs_value = fabsf(value);
        sum_abs += abs_value;
        sum_sq += value * value;
        if (value < min_value) {
            min_value = value;
        }
        if (value > max_value) {
            max_value = value;
        }
        if (abs_value > max_abs) {
            max_abs = abs_value;
        }
    }

    float mean_sq = sum_sq / (float)n;
    float std = sqrtf(mean_sq + 1e-12f);
    float threshold = fmaxf(0.05f * std, 1e-9f);

    float wl_acc = 0.0f;
    float wamp_count = 0.0f;
    float zc_count = 0.0f;
    float ssc_count = 0.0f;
    float tkeo_acc = 0.0f;

    for (size_t idx = 1; idx < n; ++idx) {
        float diff = scratch[idx] - scratch[idx - 1U];
        wl_acc += fabsf(diff);
        if (fabsf(diff) > threshold) {
            wamp_count += 1.0f;
        }
        if ((scratch[idx - 1U] * scratch[idx] < 0.0f) && (fabsf(diff) > threshold)) {
            zc_count += 1.0f;
        }
    }

    for (size_t idx = 1; idx + 1U < n; ++idx) {
        float diff_prev = scratch[idx] - scratch[idx - 1U];
        float diff_next = scratch[idx + 1U] - scratch[idx];
        if ((diff_prev * diff_next < 0.0f) && (fabsf(diff_prev - diff_next) > threshold)) {
            ssc_count += 1.0f;
        }
        tkeo_acc += scratch[idx] * scratch[idx] - scratch[idx - 1U] * scratch[idx + 1U];
    }

    out_features[0] = sum_abs / (float)n;
    out_features[1] = sqrtf(mean_sq);
    out_features[2] = std;
    out_features[3] = logf(mean_sq + 1e-12f);
    out_features[4] = wl_acc / (float)(n - 1U);
    out_features[5] = zc_count / (float)(n - 1U);
    out_features[6] = ssc_count / (float)(n - 2U);
    out_features[7] = wamp_count / (float)(n - 1U);
    out_features[8] = max_value - min_value;
    out_features[9] = max_abs;
    out_features[10] = sum_abs / (float)n;
    out_features[11] = tkeo_acc / (float)(n - 2U);

    if (ar_order > 0) {
        emg_rt_ar_yule_walker(scratch, n, ar_order, out_features + EMG_RT_BASE_FEATURES_1CH);
    }
    return emg_rt_feature_count_1ch(ar_order);
}

static inline void emg_rt_copy_interleaved_channel(
    const float *input,
    size_t n,
    size_t num_channels,
    size_t channel_idx,
    float *out
) {
    for (size_t sample_idx = 0; sample_idx < n; ++sample_idx) {
        out[sample_idx] = input[sample_idx * num_channels + channel_idx];
    }
}

static inline void emg_rt_normalize_interleaved_channels(
    float *block,
    size_t n,
    size_t num_channels,
    emg_rt_clip_normalize_mode_t mode,
    float *channel,
    float *work
) {
    if (mode == EMG_RT_CLIP_NORMALIZE_NONE) {
        return;
    }

    for (size_t channel_idx = 0; channel_idx < num_channels; ++channel_idx) {
        emg_rt_copy_interleaved_channel(block, n, num_channels, channel_idx, channel);
        float median = emg_rt_center_value(channel, n, EMG_RT_CENTER_MEDIAN, work);
        for (size_t sample_idx = 0; sample_idx < n; ++sample_idx) {
            size_t offset = sample_idx * num_channels + channel_idx;
            block[offset] -= median;
            channel[sample_idx] = block[offset];
        }

        float scale = 1.0f;
        if (mode == EMG_RT_CLIP_NORMALIZE_STD || mode == EMG_RT_CLIP_NORMALIZE_ROBUST) {
            float mean = 0.0f;
            for (size_t sample_idx = 0; sample_idx < n; ++sample_idx) {
                mean += channel[sample_idx];
            }
            mean /= (float)n;

            float var_acc = 0.0f;
            for (size_t sample_idx = 0; sample_idx < n; ++sample_idx) {
                float centered = channel[sample_idx] - mean;
                var_acc += centered * centered;
            }
            scale = sqrtf(var_acc / (float)n);
        }

        scale += 1e-9f;
        for (size_t sample_idx = 0; sample_idx < n; ++sample_idx) {
            size_t offset = sample_idx * num_channels + channel_idx;
            block[offset] /= scale;
        }
    }
}

static inline float emg_rt_corr_interleaved(
    const float *block,
    size_t n,
    size_t num_channels,
    size_t left_channel,
    size_t right_channel
) {
    float left_mean = 0.0f;
    float right_mean = 0.0f;
    for (size_t sample_idx = 0; sample_idx < n; ++sample_idx) {
        left_mean += block[sample_idx * num_channels + left_channel];
        right_mean += block[sample_idx * num_channels + right_channel];
    }
    left_mean /= (float)n;
    right_mean /= (float)n;

    float cross = 0.0f;
    float left_sq = 0.0f;
    float right_sq = 0.0f;
    for (size_t sample_idx = 0; sample_idx < n; ++sample_idx) {
        float left = block[sample_idx * num_channels + left_channel] - left_mean;
        float right = block[sample_idx * num_channels + right_channel] - right_mean;
        cross += left * right;
        left_sq += left * left;
        right_sq += right * right;
    }

    float left_std = sqrtf(left_sq / (float)n);
    float right_std = sqrtf(right_sq / (float)n);
    if (left_std <= 1e-9f || right_std <= 1e-9f) {
        return 0.0f;
    }
    return cross / (sqrtf(left_sq * right_sq) + 1e-12f);
}

static inline size_t emg_rt_append_segment_features(
    const float *block,
    size_t n,
    size_t num_channels,
    int segment_bins,
    float *out_features
) {
    if (segment_bins <= 0) {
        return 0U;
    }

    size_t out_idx = 0U;
    size_t base = n / (size_t)segment_bins;
    size_t remainder = n % (size_t)segment_bins;
    size_t start = 0U;
    for (int segment_idx = 0; segment_idx < segment_bins; ++segment_idx) {
        size_t segment_len = base + ((size_t)segment_idx < remainder ? 1U : 0U);
        for (size_t channel_idx = 0; channel_idx < num_channels; ++channel_idx) {
            float mav = 0.0f;
            float rms = 0.0f;
            float std = 0.0f;
            if (segment_len > 0U) {
                float mean = 0.0f;
                for (size_t sample_idx = 0; sample_idx < segment_len; ++sample_idx) {
                    float value = block[(start + sample_idx) * num_channels + channel_idx];
                    float abs_value = fabsf(value);
                    mav += abs_value;
                    rms += value * value;
                    mean += value;
                }
                mav /= (float)segment_len;
                rms = sqrtf(rms / (float)segment_len);
                mean /= (float)segment_len;

                float var_acc = 0.0f;
                for (size_t sample_idx = 0; sample_idx < segment_len; ++sample_idx) {
                    float value = block[(start + sample_idx) * num_channels + channel_idx] - mean;
                    var_acc += value * value;
                }
                std = sqrtf(var_acc / (float)segment_len);
            }
            out_features[out_idx++] = mav;
            out_features[out_idx++] = rms;
            out_features[out_idx++] = std;
        }
        start += segment_len;
    }
    return out_idx;
}

static inline size_t emg_rt_extract_features(
    const float *input,
    size_t n,
    size_t num_channels,
    int ar_order,
    emg_rt_center_mode_t center_mode,
    emg_rt_clip_normalize_mode_t clip_normalize,
    int segment_bins,
    float *scratch,
    float *out_features
) {
    if (n < 3U || num_channels == 0U) {
        return 0U;
    }
    if (ar_order < 0) {
        ar_order = 0;
    }
    if (ar_order > EMG_RT_MAX_AR_ORDER) {
        ar_order = EMG_RT_MAX_AR_ORDER;
    }
    if (segment_bins < 0) {
        segment_bins = 0;
    }

    float *block = scratch;
    float *channel = scratch + (n * num_channels);
    float *work = channel + n;
    size_t total_values = n * num_channels;
    for (size_t idx = 0; idx < total_values; ++idx) {
        block[idx] = input[idx];
    }

    emg_rt_normalize_interleaved_channels(block, n, num_channels, clip_normalize, channel, work);

    size_t out_idx = 0U;
    size_t per_channel_count = emg_rt_feature_count_1ch(ar_order);
    for (size_t channel_idx = 0; channel_idx < num_channels; ++channel_idx) {
        emg_rt_copy_interleaved_channel(block, n, num_channels, channel_idx, channel);
        out_idx += emg_rt_extract_features_1ch(
            channel,
            n,
            ar_order,
            center_mode,
            work,
            out_features + out_idx
        );
    }

    const size_t base_indices[3] = {1U, 0U, 4U};
    for (size_t left = 0; left < num_channels; ++left) {
        for (size_t right = left + 1U; right < num_channels; ++right) {
            for (size_t base_idx = 0; base_idx < 3U; ++base_idx) {
                size_t feature_idx = base_indices[base_idx];
                float left_value = out_features[left * per_channel_count + feature_idx];
                float right_value = out_features[right * per_channel_count + feature_idx];
                out_features[out_idx++] = left_value / (right_value + 1e-9f);
                out_features[out_idx++] = (left_value - right_value) / (left_value + right_value + 1e-9f);
            }
            out_features[out_idx++] = emg_rt_corr_interleaved(block, n, num_channels, left, right);
        }
    }

    out_idx += emg_rt_append_segment_features(
        block,
        n,
        num_channels,
        segment_bins,
        out_features + out_idx
    );
    return out_idx;
}
