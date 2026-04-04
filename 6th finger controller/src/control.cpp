#include "control.h"
#include <math.h>

static const int VIBRO_CHANNEL = 4;

float Control::smooth(float prev, float cur, float alpha)
{
    return prev * (1.0f - alpha) + cur * alpha;
}

int Control::readAdcAvg(uint8_t pin, int samples)
{
    if (samples <= 1)
        return analogRead(pin);

    uint32_t sum = 0;
    for (int i = 0; i < samples; ++i)
    {
        sum += analogRead(pin);
        delayMicroseconds(120);
    }

    return (int)(sum / samples);
}

float Control::readResistance(uint8_t pin, uint32_t pullupOhm)
{
    const float VCC = 3.3f;

    int raw = analogRead(pin);
    if (raw <= 0)
        return 1e9f;

    float v = (raw / 4095.0f) * VCC;
    if (v < 0.001f)
        return 1e9f;

    return pullupOhm * (VCC / v - 1.0f);
}

float Control::resistanceFromAdc(float adc, uint32_t pullupOhm) const
{
    const float VCC = 3.3f;

    if (pullupOhm == 0)
        return INFINITY;

    if (adc <= FSR_OPEN_ADC)
        return INFINITY;

    float v = (adc / 4095.0f) * VCC;

    if (v <= 0.005f)
        return INFINITY;

    return pullupOhm * (VCC / v - 1.0f);
}

float Control::sanitizeResistanceForTelemetry(float resistanceOhm) const
{
    if (!isfinite(resistanceOhm) || resistanceOhm > FSR_MAX_REPORT_OHM)
        return FSR_MAX_REPORT_OHM;

    if (resistanceOhm < 0.0f)
        return 0.0f;

    return resistanceOhm;
}

float Control::fsrToNewton(float resistanceOhm)
{
    if (!isfinite(resistanceOhm))
        return 0.0f;

    const float R_NO_TOUCH = 150000.0f;
    const float R_FULL_SCALE = 1200.0f;

    if (resistanceOhm >= R_NO_TOUCH)
        return 0.0f;

    if (resistanceOhm <= R_FULL_SCALE)
        return current.fsrHardMaxN;

    float x = (logf(R_NO_TOUCH) - logf(resistanceOhm)) /
              (logf(R_NO_TOUCH) - logf(R_FULL_SCALE));

    if (x < 0.0f)
        x = 0.0f;
    if (x > 1.0f)
        x = 1.0f;

    x = powf(x, 2.2f);

    return x * current.fsrHardMaxN;
}

static float clampf(float v, float a, float b)
{
    if (v < a)
        return a;
    if (v > b)
        return b;
    return v;
}

float Control::flexToAngle(float rohm, int idx) const
{
    const FlexSettings &fCfg = current.flex[idx];
    const ServoSettings &sCfg = current.servo[idx];

    const float minA = sCfg.servoMinDeg;
    const float maxA = sCfg.servoMaxDeg;

    const float st = (float)fCfg.flexStraightOhm;
    const float bd = (float)fCfg.flexBendOhm;

    if (bd <= st + 1.0f)
    {
        return (rohm <= st) ? maxA : minA;
    }

    if (rohm <= st)
        return maxA;

    if (rohm >= bd)
        return minA;

    float t = (rohm - st) / (bd - st);
    t = clampf(t, 0.0f, 1.0f);

    return maxA - t * (maxA - minA);
}

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

void Control::updateServo(float targetAngleDeg, int idx)
{
    ServoSettings &cfg = current.servo[idx];

    tele.servoTargetDeg[idx] = targetAngleDeg;

    if (cfg.servoManual == ServoManualMode::Manual)
        targetAngleDeg = cfg.servoManualDeg;

    uint32_t now = millis();
    float dt = (now - lastServoUpdate[idx]) / 1000.0f;
    if (dt <= 0.0001f)
        dt = 0.0001f;
    lastServoUpdate[idx] = now;

    float maxStep = cfg.servoMaxSpeedDegPerSec * dt;
    float diff = targetAngleDeg - servoAngleDeg[idx];

    if (fabsf(diff) > maxStep)
        servoAngleDeg[idx] += (diff > 0 ? maxStep : -maxStep);
    else
        servoAngleDeg[idx] = targetAngleDeg;

    tele.servoCurrentDeg[idx] = servoAngleDeg[idx];
    tele.servoSpeedDps[idx] = fabsf(diff) / dt;

    if (servoAngleDeg[idx] >= cfg.servoMaxDeg - 1.0f)
    {
        if (servos[idx].attached())
            servos[idx].detach();
    }
    else
    {
        if (!servos[idx].attached())
            servos[idx].attach(cfg.servoPin, 500, 2400);
        servos[idx].write((int)servoAngleDeg[idx]);
    }
}

void Control::updateVibro(uint8_t duty)
{
    vibroDuty = duty;
    tele.vibroDuty = duty;
    tele.vibroActive = duty > 0;
    tele.vibroMode = current.vibroMode;

    ledcWrite(VIBRO_CHANNEL, duty);
}

void Control::pushFlexHistory(int idx, float v)
{
    uint8_t pos = flexHistPos[idx];
    flexHist[idx][pos] = v;

    pos++;
    if (pos >= FLEX_HISTORY)
        pos = 0;
    flexHistPos[idx] = pos;

    if (flexHistCount[idx] < FLEX_HISTORY)
        flexHistCount[idx]++;
}

static float medianOfSmall(const float *arr, int n)
{
    float tmp[5];
    for (int i = 0; i < n; ++i)
        tmp[i] = arr[i];

    for (int i = 1; i < n; ++i)
    {
        float key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key)
        {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }

    if (n <= 0)
        return 0.0f;
    return tmp[n / 2];
}

float Control::medianFlexHistory(int idx) const
{
    int n = (int)flexHistCount[idx];
    if (n <= 0)
        return 0.0f;

    float tmp[5];
    for (int i = 0; i < n; ++i)
        tmp[i] = flexHist[idx][i];

    return medianOfSmall(tmp, n);
}

void Control::setupHardware()
{
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    pinMode(current.vibroPin, OUTPUT);
    ledcSetup(VIBRO_CHANNEL, current.vibroFreqHz, 8);
    ledcAttachPin(current.vibroPin, VIBRO_CHANNEL);
    ledcWrite(VIBRO_CHANNEL, 0);

    uint32_t now = millis();
    for (int i = 0; i < NUM_PAIRS; ++i)
    {
        if (servos[i].attached())
            servos[i].detach();
        servoAngleDeg[i] = current.servo[i].servoMaxDeg;
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
    }

    fsrFiltered = 0.0f;
    fsrRaw = 0.0f;
    fsrAdcFiltered = 0.0f;
    vibroDuty = 0;
}

void Control::begin(const Settings &s)
{
    current = s;
    setupHardware();
}

void Control::reconfigure(const Settings &s)
{
    current = s;
    setupHardware();
}

void Control::update()
{
    int fsrAdcRaw = readAdcAvg(current.fsrPin, FSR_SAMPLES);

    float alpha = (fsrAdcRaw > fsrAdcFiltered) ? FSR_PRESS_ALPHA : FSR_RELEASE_ALPHA;
    fsrAdcFiltered = smooth(fsrAdcFiltered, (float)fsrAdcRaw, alpha);

    fsrRaw = resistanceFromAdc((float)fsrAdcRaw, current.fsrPullupOhm);
    fsrFiltered = resistanceFromAdc(fsrAdcFiltered, current.fsrPullupOhm);

    tele.fsrRawOhm = sanitizeResistanceForTelemetry(fsrRaw);
    tele.fsrFilteredOhm = sanitizeResistanceForTelemetry(fsrFiltered);

    float forceN = fsrToNewton(fsrFiltered);
    tele.fsrForceN = forceN;

    for (int i = 0; i < NUM_PAIRS; ++i)
    {
        const FlexSettings &cfg = current.flex[i];
        const ServoSettings &sCfg = current.servo[i];

        if (cfg.flexPin == 0xFF || sCfg.servoPin == 0xFF ||
            cfg.flexPin == 0 || sCfg.servoPin == 0)
        {
            if (servos[i].attached())
                servos[i].detach();
            continue;
        }

        float rNew = readResistance(cfg.flexPin, cfg.flexPullupOhm);

        float rSample = rNew;
        const int histN = (int)flexHistCount[i];

        if (histN >= 3)
        {
            float med = medianFlexHistory(i);
            if (med > 1.0f)
            {
                bool outlier = (rNew > med * 2.0f) || (rNew < med * 0.5f);
                if (outlier)
                {
                    flexOutlierStrikes[i]++;
                    if (flexOutlierStrikes[i] < 2)
                    {
                        rSample = med;
                    }
                    else
                    {
                        flexOutlierStrikes[i] = 0;
                        rSample = rNew;
                        pushFlexHistory(i, rSample);
                    }
                }
                else
                {
                    flexOutlierStrikes[i] = 0;
                    rSample = rNew;
                    pushFlexHistory(i, rSample);
                }
            }
            else
            {
                flexOutlierStrikes[i] = 0;
                rSample = rNew;
                pushFlexHistory(i, rSample);
            }
        }
        else
        {
            flexOutlierStrikes[i] = 0;
            rSample = rNew;
            pushFlexHistory(i, rSample);
        }

        int pct = (int)cfg.flexTolerancePct;
        if (pct < 1)
            pct = 1;
        if (pct > 50)
            pct = 50;

        if (!flexStableInit[i])
        {
            flexStableInit[i] = true;
            flexStableOhm[i] = rSample;
        }

        float stable = flexStableOhm[i];
        float tol = fabsf(stable) * (pct / 100.0f);
        if (tol < 1.0f)
            tol = 1.0f;

        float diff = rSample - stable;

        if (fabsf(diff) <= tol)
        {
        }
        else
        {
            float sign = (diff >= 0.0f) ? 1.0f : -1.0f;

            float target = stable + (diff - sign * tol);

            float absMove = fabsf(target - stable);

            float a = 0.20f;
            if (absMove > 10.0f * tol)
                a = 0.45f;
            else if (absMove > 4.0f * tol)
                a = 0.30f;

            flexStableOhm[i] = smooth(stable, target, a);
        }

        float rFilt = flexStableOhm[i];

        flexFiltered[i] = rFilt;
        flexRaw[i] = rNew;

        tele.flexRawOhm[i] = sanitizeResistanceForTelemetry(flexRaw[i]);
        tele.flexFilteredOhm[i] = sanitizeResistanceForTelemetry(flexFiltered[i]);

        float targetAngle = flexToAngle(flexFiltered[i], i);
        updateServo(targetAngle, i);
    }

    bool pulseMode = false;
    uint8_t duty = computeVibroDuty(forceN, pulseMode);
    updateVibro(duty);
}