#include "emg_engine.h"
#include <math.h>

void EmgEngine::setAdcReader(AdcReadFn fn, void *ctx)
{
    adcReadFn = fn;
    adcReadCtx = ctx;
}

bool EmgEngine::isValidPin(uint8_t pin) const
{
    return pin != 0 && pin != UNUSED_PIN;
}

void EmgEngine::resetPairState(int idx)
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return;

    fingerState[idx] = FingerState::Extended;
    interpActive[idx] = false;
    interpStartedMs[idx] = 0;
    interpLastActiveMs[idx] = 0;
    interpSamples[idx] = 0;

    for (int ch = 0; ch < 3; ++ch)
        interpPeak[idx][ch] = 0.0f;

    cooldownUntilMs[idx] = 0;
    bendCounter[idx] = 0;
    unfoldCounter[idx] = 0;
    targetBent[idx] = false;

    teleEventHoldUntilMs[idx] = 0;
    teleActionHoldUntilMs[idx] = 0;
}

void EmgEngine::setTelemetryInactive(ControlTelemetry &tele, int idx) const
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return;

    tele.emgMode[idx] = -1;
    tele.emgChannelCount[idx] = -1;
    tele.emgEvent[idx] = (int8_t)EmgEvent::None;
    tele.emgAction[idx] = (int8_t)EmgAction::None;
    tele.emgCooldownMs[idx] = -1;
    tele.emgBendProgress[idx] = -1;
    tele.emgUnfoldProgress[idx] = -1;
    tele.emgCh0[idx] = NAN;
    tele.emgCh1[idx] = NAN;
    tele.emgCh2[idx] = NAN;
}

void EmgEngine::reset(const Settings &s, ControlTelemetry &tele)
{
    (void)s;

    for (int i = 0; i < NUM_PAIRS; ++i)
    {
        resetPairState(i);
        setTelemetryInactive(tele, i);
    }
}

bool EmgEngine::isPairConfigured(const Settings &s, int idx) const
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return false;
    return s.emg[idx].activePinsValid();
}

bool EmgEngine::shouldUseMock(const Settings &s, int idx) const
{
    if (!isPairConfigured(s, idx))
        return false;

    const EmgSettings &cfg = s.emg[idx];
    const uint8_t pins[3] = {cfg.pin0, cfg.pin1, cfg.pin2};

    for (uint8_t ch = 0; ch < cfg.channels; ++ch)
    {
        const uint8_t p = pins[ch];
        if (p != 34 && p != 35 && p != 36)
            return false;
    }

    return true;
}

void EmgEngine::makeMockSnapshot(const EmgSettings &cfg, uint32_t nowMs, float outRaw[3]) const
{
    static const uint16_t BASELINE = 1220;
    static const uint16_t LOW_A = 520;
    static const uint16_t MID_A = 980;
    static const uint16_t HIGH_A = 2100;

    struct MockStep
    {
        MockMotion motion;
        uint16_t durationMs;
    };

    static const MockStep bendOtherSeq[] = {
        {MockMotion::Idle, 850},
        {MockMotion::FingerBend, 230},
        {MockMotion::Idle, 200},
        {MockMotion::FingerUnfold, 420},
        {MockMotion::Idle, 180},
        {MockMotion::Other, 320},
        {MockMotion::Idle, 180},
        {MockMotion::Other, 340},
        {MockMotion::Idle, 180},
        {MockMotion::FingerBend, 240},
        {MockMotion::Idle, 900},
    };

    static const MockStep directionalSeq[] = {
        {MockMotion::Idle, 800},
        {MockMotion::FingerBend, 230},
        {MockMotion::Idle, 180},
        {MockMotion::Other, 320},
        {MockMotion::Idle, 180},
        {MockMotion::FingerUnfold, 430},
        {MockMotion::Idle, 180},
        {MockMotion::Other, 320},
        {MockMotion::Idle, 180},
        {MockMotion::FingerBend, 230},
        {MockMotion::Idle, 180},
        {MockMotion::FingerUnfold, 430},
        {MockMotion::Idle, 900},
    };

    const MockStep *seq = (cfg.mode == EmgMode::Directional) ? directionalSeq : bendOtherSeq;
    const size_t seqCount =
        (cfg.mode == EmgMode::Directional) ? (sizeof(directionalSeq) / sizeof(directionalSeq[0]))
                                           : (sizeof(bendOtherSeq) / sizeof(bendOtherSeq[0]));

    uint32_t period = 0;
    for (size_t i = 0; i < seqCount; ++i)
        period += seq[i].durationMs;

    uint32_t t = (period > 0) ? (nowMs % period) : 0;
    MockMotion motion = MockMotion::Idle;
    uint16_t phaseMs = 0;
    uint16_t phaseDur = 1;

    for (size_t i = 0; i < seqCount; ++i)
    {
        if (t < seq[i].durationMs)
        {
            motion = seq[i].motion;
            phaseMs = (uint16_t)t;
            phaseDur = seq[i].durationMs;
            break;
        }
        t -= seq[i].durationMs;
    }

    float env = 0.0f;
    if (motion != MockMotion::Idle)
    {
        float k = (float)phaseMs / (float)phaseDur;
        if (motion == MockMotion::FingerBend)
        {
            if (k < 0.25f)
                env = k / 0.25f;
            else if (k < 0.70f)
                env = 1.0f;
            else
                env = 1.0f - (k - 0.70f) / 0.30f;
        }
        else if (motion == MockMotion::FingerUnfold)
        {
            if (k < 0.20f)
                env = k / 0.20f;
            else if (k < 0.45f)
                env = 1.0f;
            else if (k < 0.60f)
                env = 0.55f;
            else if (k < 0.82f)
                env = 1.0f;
            else
                env = 1.0f - (k - 0.82f) / 0.18f;
        }
        else if (motion == MockMotion::Other)
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

    for (int ch = 0; ch < 3; ++ch)
        outRaw[ch] = (float)BASELINE;

    const int channels = cfg.channels;

    if (channels >= 3)
    {
        switch (motion)
        {
        case MockMotion::FingerBend:
            outRaw[0] = BASELINE + HIGH_A * env;
            outRaw[1] = BASELINE + LOW_A * env * 0.25f;
            outRaw[2] = BASELINE + LOW_A * env * 0.12f;
            break;
        case MockMotion::FingerUnfold:
            outRaw[0] = BASELINE + LOW_A * env * 0.20f;
            outRaw[1] = BASELINE + HIGH_A * env;
            outRaw[2] = BASELINE + LOW_A * env * 0.12f;
            break;
        case MockMotion::Other:
            outRaw[0] = BASELINE + LOW_A * env * 0.12f;
            outRaw[1] = BASELINE + LOW_A * env * 0.12f;
            outRaw[2] = BASELINE + HIGH_A * env;
            break;
        case MockMotion::Idle:
        default:
            break;
        }
    }
    else if (channels == 2)
    {
        switch (motion)
        {
        case MockMotion::FingerBend:
            outRaw[0] = BASELINE + HIGH_A * env;
            outRaw[1] = BASELINE + LOW_A * env * 0.22f;
            break;
        case MockMotion::FingerUnfold:
            outRaw[0] = BASELINE + LOW_A * env * 0.22f;
            outRaw[1] = BASELINE + HIGH_A * env;
            break;
        case MockMotion::Other:
            outRaw[0] = BASELINE + MID_A * env;
            outRaw[1] = BASELINE + MID_A * env * 0.93f;
            break;
        case MockMotion::Idle:
        default:
            break;
        }
    }
    else
    {
        switch (motion)
        {
        case MockMotion::FingerBend:
            outRaw[0] = BASELINE + HIGH_A * env;
            break;
        case MockMotion::FingerUnfold:
            outRaw[0] = BASELINE + (HIGH_A * 0.85f) * env;
            break;
        case MockMotion::Other:
            outRaw[0] = BASELINE + MID_A * env;
            break;
        case MockMotion::Idle:
        default:
            break;
        }
    }

    for (int ch = 0; ch < channels; ++ch)
    {
        int wobble = ((int)(nowMs / 17U + (uint32_t)(ch * 11)) % 25) - 12;
        float v = outRaw[ch] + (float)wobble;
        if (v < 0.0f)
            v = 0.0f;
        if (v > 4095.0f)
            v = 4095.0f;
        outRaw[ch] = v;
    }
}

void EmgEngine::readChannels(int idx, const Settings &s, uint32_t nowMs, float outRaw[3]) const
{
    outRaw[0] = NAN;
    outRaw[1] = NAN;
    outRaw[2] = NAN;

    if (!isPairConfigured(s, idx))
        return;

    const EmgSettings &cfg = s.emg[idx];
    const int channels = cfg.channels;

    if (shouldUseMock(s, idx))
    {
        makeMockSnapshot(cfg, nowMs, outRaw);
        for (int ch = channels; ch < 3; ++ch)
            outRaw[ch] = NAN;
        return;
    }

    if (adcReadFn == nullptr)
        return;

    const uint8_t pins[3] = {cfg.pin0, cfg.pin1, cfg.pin2};
    for (int ch = 0; ch < channels && ch < 3; ++ch)
    {
        if (!isValidPin(pins[ch]))
            continue;
        outRaw[ch] = (float)adcReadFn(adcReadCtx, pins[ch], EMG_SAMPLES);
    }
}

EmgEngine::MockMotion EmgEngine::classifyWindow(int idx, int channels) const
{
    if (channels < 1)
        return MockMotion::Idle;

    const float samples = (float)interpSamples[idx];
    if (samples < 1.0f)
        return MockMotion::Idle;

    const float durationMs =
        (float)(interpLastActiveMs[idx] - interpStartedMs[idx] + 1U);

    const float p0 = interpPeak[idx][0];
    const float p1 = interpPeak[idx][1];
    const float p2 = interpPeak[idx][2];

    if (channels >= 3)
    {
        if (p0 >= p1 && p0 >= p2)
            return MockMotion::FingerBend;
        if (p1 >= p0 && p1 >= p2)
            return MockMotion::FingerUnfold;
        return MockMotion::Other;
    }

    if (channels == 2)
    {
        const float d = fabsf(p0 - p1);
        if (d < 240.0f)
            return MockMotion::Other;
        return (p0 > p1) ? MockMotion::FingerBend : MockMotion::FingerUnfold;
    }

    if (p0 < 1650.0f)
        return MockMotion::Idle;

    if (durationMs >= 360.0f)
        return MockMotion::FingerUnfold;

    if (p0 < 2500.0f || durationMs >= 270.0f)
        return MockMotion::Other;

    return MockMotion::FingerBend;
}

EmgEvent EmgEngine::motionToEvent(EmgMode mode, MockMotion motion) const
{
    switch (motion)
    {
    case MockMotion::FingerBend:
        return EmgEvent::Bend;

    case MockMotion::FingerUnfold:
        return (mode == EmgMode::Directional) ? EmgEvent::Unfold : EmgEvent::Bend;

    case MockMotion::Other:
        return EmgEvent::Other;

    case MockMotion::Idle:
    default:
        return EmgEvent::None;
    }
}

void EmgEngine::setTeleEvent(ControlTelemetry &tele, int idx, EmgEvent event, uint32_t holdMs)
{
    tele.emgEvent[idx] = (int8_t)event;
    teleEventHoldUntilMs[idx] = millis() + holdMs;
}

void EmgEngine::setTeleAction(ControlTelemetry &tele, int idx, EmgAction action, uint32_t holdMs)
{
    tele.emgAction[idx] = (int8_t)action;
    teleActionHoldUntilMs[idx] = millis() + holdMs;
}

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

EmgEvent EmgEngine::processInterpreter(int idx, const Settings &s, const float raw[3], uint32_t nowMs)
{
    const EmgSettings &cfg = s.emg[idx];
    const int channels = cfg.channels;

    float maxRaw = 0.0f;
    for (int ch = 0; ch < channels && ch < 3; ++ch)
    {
        if (isfinite(raw[ch]) && raw[ch] > maxRaw)
            maxRaw = raw[ch];
    }

    const bool activeNow = maxRaw >= ACTIVE_RAW_THRESHOLD;
    const bool wasActive = interpActive[idx];

    if (activeNow)
    {
        if (!wasActive)
        {
            interpActive[idx] = true;
            interpStartedMs[idx] = nowMs;
            interpLastActiveMs[idx] = nowMs;
            interpSamples[idx] = 0;
            for (int ch = 0; ch < 3; ++ch)
                interpPeak[idx][ch] = 0.0f;
        }

        interpLastActiveMs[idx] = nowMs;
        if (interpSamples[idx] < 65530)
            interpSamples[idx]++;

        for (int ch = 0; ch < channels && ch < 3; ++ch)
        {
            if (!isfinite(raw[ch]))
                continue;
            if (raw[ch] > interpPeak[idx][ch])
                interpPeak[idx][ch] = raw[ch];
        }

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

    MockMotion motion = classifyWindow(idx, channels);
    interpSamples[idx] = 0;
    return motionToEvent(cfg.mode, motion);
}

float EmgEngine::targetAngleForPair(const Settings &s, int idx) const
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return 0.0f;

    const ServoSettings &servoCfg = s.servo[idx];
    const EmgSettings &emgCfg = s.emg[idx];

    const float bendAngle = emgCfg.reverseDirection ? servoCfg.servoMaxDeg : servoCfg.servoMinDeg;
    const float unfoldAngle = emgCfg.reverseDirection ? servoCfg.servoMinDeg : servoCfg.servoMaxDeg;

    return targetBent[idx] ? bendAngle : unfoldAngle;
}

void EmgEngine::applyEvent(int idx, const Settings &s, EmgEvent event, uint32_t nowMs, ControlTelemetry &tele)
{
    const EmgSettings &cfg = s.emg[idx];

    tele.emgBendProgress[idx] = (int8_t)bendCounter[idx];
    tele.emgUnfoldProgress[idx] = (int8_t)unfoldCounter[idx];
    tele.emgCooldownMs[idx] =
        (nowMs < cooldownUntilMs[idx]) ? (int16_t)(cooldownUntilMs[idx] - nowMs) : 0;

    if (event == EmgEvent::None)
        return;

    setTeleEvent(tele, idx, event);

    if (cfg.mode == EmgMode::Directional)
    {
        if (event == EmgEvent::Bend)
        {
            fingerState[idx] = FingerState::Bent;
            targetBent[idx] = true;
            bendCounter[idx] = 0;
            unfoldCounter[idx] = 0;
            tele.emgBendProgress[idx] = 0;
            tele.emgUnfoldProgress[idx] = 0;
            setTeleAction(tele, idx, EmgAction::Bend);
        }
        else if (event == EmgEvent::Unfold)
        {
            fingerState[idx] = FingerState::Extended;
            targetBent[idx] = false;
            bendCounter[idx] = 0;
            unfoldCounter[idx] = 0;
            tele.emgBendProgress[idx] = 0;
            tele.emgUnfoldProgress[idx] = 0;
            setTeleAction(tele, idx, EmgAction::Unfold);
        }
        return;
    }

    if (fingerState[idx] == FingerState::Extended)
    {
        if (event == EmgEvent::Bend)
        {
            if (bendCounter[idx] < 5)
                bendCounter[idx]++;

            tele.emgBendProgress[idx] = (int8_t)bendCounter[idx];

            if (bendCounter[idx] >= cfg.bendFullMoves)
            {
                fingerState[idx] = FingerState::Bent;
                targetBent[idx] = true;
                bendCounter[idx] = 0;
                unfoldCounter[idx] = 0;
                cooldownUntilMs[idx] = nowMs + (uint32_t)cfg.minSwitchDelaySec * 1000UL;
                tele.emgBendProgress[idx] = 0;
                tele.emgUnfoldProgress[idx] = 0;
                tele.emgCooldownMs[idx] = (int16_t)(cooldownUntilMs[idx] - nowMs);
                setTeleAction(tele, idx, EmgAction::Bend);
            }
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

    if (event == EmgEvent::Bend)
    {
        if (unfoldCounter[idx] < 5)
            unfoldCounter[idx]++;

        tele.emgUnfoldProgress[idx] = (int8_t)unfoldCounter[idx];

        if (unfoldCounter[idx] >= cfg.unfoldFullMoves)
        {
            fingerState[idx] = FingerState::Extended;
            targetBent[idx] = false;
            bendCounter[idx] = 0;
            unfoldCounter[idx] = 0;
            tele.emgBendProgress[idx] = 0;
            tele.emgUnfoldProgress[idx] = 0;
            setTeleAction(tele, idx, EmgAction::Unfold);
        }
    }
}

void EmgEngine::updatePair(int idx, const Settings &s, uint32_t nowMs, ControlTelemetry &tele)
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return;

    refreshTeleTimers(tele, idx, nowMs);

    const EmgSettings &cfg = s.emg[idx];

    tele.emgSource[idx] = (int8_t)InputSource::Emg;
    tele.emgMode[idx] = (int8_t)cfg.mode;
    tele.emgChannelCount[idx] = (int8_t)cfg.channels;
    tele.emgBendProgress[idx] = (int8_t)bendCounter[idx];
    tele.emgUnfoldProgress[idx] = (int8_t)unfoldCounter[idx];
    tele.emgCooldownMs[idx] =
        (nowMs < cooldownUntilMs[idx]) ? (int16_t)(cooldownUntilMs[idx] - nowMs) : 0;

    if (!isPairConfigured(s, idx))
    {
        tele.emgCh0[idx] = NAN;
        tele.emgCh1[idx] = NAN;
        tele.emgCh2[idx] = NAN;
        return;
    }

    float raw[3];
    readChannels(idx, s, nowMs, raw);

    tele.emgCh0[idx] = raw[0];
    tele.emgCh1[idx] = raw[1];
    tele.emgCh2[idx] = raw[2];

    EmgEvent event = processInterpreter(idx, s, raw, nowMs);
    applyEvent(idx, s, event, nowMs, tele);
}
