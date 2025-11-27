#include "control.h"

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

float Control::flexToAngle(float rohm) const
{
    const float minA = current.servoMinDeg;
    const float maxA = current.servoMaxDeg;

    if (rohm <= current.flexStraightOhm)
        return maxA;

    if (rohm >= current.flexBendOhm)
        return minA;

    float t = (rohm - current.flexStraightOhm) /
              float(current.flexBendOhm - current.flexStraightOhm);
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

void Control::updateServo(float targetAngleDeg)
{
    tele.servoTargetDeg = targetAngleDeg;

    if (current.servoManual == ServoManualMode::Manual)
        targetAngleDeg = current.servoManualDeg;

    uint32_t now = millis();
    float dt = (now - lastServoUpdate) / 1000.0f;
    if (dt <= 0.0001f)
        dt = 0.0001f;
    lastServoUpdate = now;

    float maxStep = current.servoMaxSpeedDegPerSec * dt;
    float diff = targetAngleDeg - servoAngleDeg;

    if (fabsf(diff) > maxStep)
        servoAngleDeg += (diff > 0 ? maxStep : -maxStep);
    else
        servoAngleDeg = targetAngleDeg;

    tele.servoCurrentDeg = servoAngleDeg;
    tele.servoSpeedDps = fabsf(diff) / dt;

    if (servoAngleDeg >= current.servoMaxDeg - 1.0f)
    {
        if (servo.attached())
            servo.detach();
    }
    else
    {
        if (!servo.attached())
            servo.attach(current.servoPin, 500, 2400);
        servo.write((int)servoAngleDeg);
    }
}

void Control::updateVibro(uint8_t duty)
{
    vibroDuty = duty;
    tele.vibroDuty = duty;
    tele.vibroActive = duty > 0;
    tele.vibroMode = current.vibroMode;

    ledcWrite(0, duty);
}

void Control::setupHardware()
{

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    pinMode(current.vibroPin, OUTPUT);
    ledcSetup(0, current.vibroFreqHz, 8);
    ledcAttachPin(current.vibroPin, 0);
    ledcWrite(0, 0);

    servo.detach();
    servoAngleDeg = current.servoMaxDeg;
    lastServoUpdate = millis();

    flexFiltered = 0.0f;
    fsrFiltered = 0.0f;
    flexRaw = 0.0f;
    fsrRaw = 0.0f;
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

    flexRaw = readResistance(current.flexPin, current.flexPullupOhm);
    fsrRaw = readResistance(current.fsrPin, current.fsrPullupOhm);

    flexFiltered = smooth(flexFiltered, flexRaw, 0.25f);
    fsrFiltered = smooth(fsrFiltered, fsrRaw, 0.25f);

    tele.flexRawOhm = flexRaw;
    tele.flexFilteredOhm = flexFiltered;
    tele.fsrRawOhm = fsrRaw;
    tele.fsrFilteredOhm = fsrFiltered;

    float forceN = fsrToNewton(fsrFiltered);
    tele.fsrForceN = forceN;

    float targetAngle = flexToAngle(flexFiltered);
    updateServo(targetAngle);

    bool pulseMode = false;
    uint8_t duty = computeVibroDuty(forceN, pulseMode);
    updateVibro(duty);
}
