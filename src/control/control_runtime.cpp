// Top-level control lifecycle.
#include "control.h"

// Apply an initial settings snapshot and initialize hardware/runtime state.
void Control::begin(const Settings &s)
{
    current = s;
    setupHardware();
}

// Rebuild runtime state after BLE changes the persisted settings model.
void Control::reconfigure(const Settings &s)
{
    current = s;
    setupHardware();
}

void Control::update()
{
    const uint32_t nowMs = millis();
    bool pairReadyForServo[NUM_PAIRS] = {};

    for (int i = 0; i < NUM_PAIRS; ++i)
    {
        const InputSource source = current.pairInput[i].inputSource;
        const ServoSettings &sCfg = current.servo[i];
        const FlexSettings &fCfg = current.flex[i];

        tele.emgSource[i] = (int8_t)source;

        // A pair without a servo is treated as inactive regardless of its input source.
        if (!isValidPin(sCfg.servoPin))
        {
            emg.setTelemetryInactive(tele, i);
            if (servos[i].attached())
                servos[i].detach();
            pairReadyForServo[i] = false;
            continue;
        }

        if (source == InputSource::Emg)
        {
            if (!emg.isPairConfigured(current, i))
            {
                emg.setTelemetryInactive(tele, i);
                if (servos[i].attached())
                    servos[i].detach();
                pairReadyForServo[i] = false;
                continue;
            }

            emg.updatePair(i, current, nowMs, tele);
            targetAngleDeg[i] = emg.targetAngleForPair(current, i);
            pairReadyForServo[i] = true;
            continue;
        }

        emg.setTelemetryInactive(tele, i);

        if (!isValidPin(fCfg.flexPin))
        {
            if (servos[i].attached())
                servos[i].detach();
            pairReadyForServo[i] = false;
            continue;
        }

        pairReadyForServo[i] = true;
    }

    processOneSlowInput(nowMs);

    for (int i = 0; i < NUM_PAIRS; ++i)
    {
        if (!pairReadyForServo[i])
            continue;
        updateServo(targetAngleDeg[i], i);
    }
}
