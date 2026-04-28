// Servo live override handling.
#include "control.h"

#include <math.h>

namespace
{
    // Local float clamp for BLE live-control angle bounds.
    float clampf(float v, float a, float b)
    {
        if (v < a)
            return a;
        if (v > b)
            return b;
        return v;
    }
}

// Apply a temporary target angle pushed from BLE live-control mode.
void Control::liveServoSet(int idx, float deg)
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return;

    float a = clampf(deg, 0.0f, 180.0f);

    liveActive[idx] = true;
    liveDeg[idx] = a;
    liveUntilMs[idx] = millis() + LIVE_TTL_MS;
}

// Clear the BLE live-control override for a single pair.
void Control::liveServoStop(int idx)
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return;
    liveActive[idx] = false;
    liveUntilMs[idx] = 0;
}

// Move one servo toward its target while respecting manual mode and speed limits.
void Control::updateServo(float targetAngleDeg, int idx)
{
    ServoSettings &cfg = current.servo[idx];

    if (cfg.servoManual == ServoManualMode::Manual)
        targetAngleDeg = cfg.servoManualDeg;

    if (liveActive[idx])
    {
        uint32_t now = millis();
        if (now <= liveUntilMs[idx])
        {
            targetAngleDeg = liveDeg[idx];
        }
        else
        {
            liveActive[idx] = false;
        }
    }

    tele.servoTargetDeg[idx] = targetAngleDeg;

    uint32_t now = millis();
    float dt = (now - lastServoUpdate[idx]) / 1000.0f;
    if (dt <= 0.0001f)
        dt = 0.0001f;
    lastServoUpdate[idx] = now;

    const float prevAngle = servoAngleDeg[idx];
    float maxStep = cfg.servoMaxSpeedDegPerSec * dt;
    float diff = targetAngleDeg - prevAngle;

    // Servo speed limiting is done in angle space to keep motion predictable.
    if (fabsf(diff) > maxStep)
        servoAngleDeg[idx] += (diff > 0 ? maxStep : -maxStep);
    else
        servoAngleDeg[idx] = targetAngleDeg;

    tele.servoCurrentDeg[idx] = servoAngleDeg[idx];
    tele.servoSpeedDps[idx] = fabsf(servoAngleDeg[idx] - prevAngle) / dt;

    // Fully opened fingers can detach the servo to reduce load and noise.
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
