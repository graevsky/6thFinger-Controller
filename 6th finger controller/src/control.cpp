#include "control.h"

static const int VIBRO_CHANNEL = 4;

float Control::smooth(float prev, float cur, float alpha)
{
    return prev * (1.0f - alpha) + cur * alpha;
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

float Control::fsrToNewton(float resistanceOhm)
{
    float kOhm = resistanceOhm / 1000.0f;
    if (kOhm < 0.001f)
        return current.fsrHardMaxN;

    float f = 1.0f / kOhm;
    if (f < 0.0f)
        f = 0.0f;
    if (f > current.fsrHardMaxN)
        f = current.fsrHardMaxN;
    return f;
}

float Control::flexToAngle(float rohm, int idx) const
{
    const FlexSettings &fCfg = current.flex[idx];
    const ServoSettings &sCfg = current.servo[idx];

    const float minA = sCfg.servoMinDeg;
    const float maxA = sCfg.servoMaxDeg;

    if (rohm <= fCfg.flexStraightOhm)
        return maxA;

    if (rohm >= fCfg.flexBendOhm)
        return minA;

    float t = (rohm - fCfg.flexStraightOhm) /
              float(fCfg.flexBendOhm - fCfg.flexStraightOhm);
    if (t < 0.0f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;

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
        if (t < 0.0f)
            t = 0.0f;
        if (t > 1.0f)
            t = 1.0f;

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
    if (t < 0.0f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;

    float base = current.vibroPulseBase;
    float amp = t * (current.vibroMaxDuty - base);

    float x = millis() / 1000.0f;
    float dutyF = base;

    if (current.vibroMode == VibroMode::Pulse)
    {
        dutyF = base + sinf(x * 2.0f * PI * 4.0f) * amp;
    }
    else
    {
        dutyF = base + amp;
    }

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
    }

    fsrFiltered = 0.0f;
    fsrRaw = 0.0f;
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
    // FSR — один на все пары
    fsrRaw = readResistance(current.fsrPin, current.fsrPullupOhm);
    fsrFiltered = smooth(fsrFiltered, fsrRaw, 0.25f);

    tele.fsrRawOhm = fsrRaw;
    tele.fsrFilteredOhm = fsrFiltered;

    float forceN = fsrToNewton(fsrFiltered);
    tele.fsrForceN = forceN;

    // Каждая пара: свой flex и своя серва
    for (int i = 0; i < NUM_PAIRS; ++i)
    {
        const FlexSettings &cfg = current.flex[i];
        const ServoSettings &sCfg = current.servo[i];

        // Неактивная пара: pin помечен как 0xFF или 0
        if (cfg.flexPin == 0xFF || sCfg.servoPin == 0xFF ||
            cfg.flexPin == 0 || sCfg.servoPin == 0)
        {
            // Серва отключена, телеметрию для неё не трогаем
            if (servos[i].attached())
                servos[i].detach();
            continue;
        }

        flexRaw[i] = readResistance(cfg.flexPin, cfg.flexPullupOhm);
        flexFiltered[i] = smooth(flexFiltered[i], flexRaw[i], 0.25f);

        tele.flexRawOhm[i] = flexRaw[i];
        tele.flexFilteredOhm[i] = flexFiltered[i];

        float targetAngle = flexToAngle(flexFiltered[i], i);
        updateServo(targetAngle, i);
    }

    bool pulseMode = false;
    uint8_t duty = computeVibroDuty(forceN, pulseMode);
    updateVibro(duty);
}
