// BLE telemetry flattening and deferred send loop.
#include "ble_app.h"

// Build the flat telemetry JSON object expected by the mobile app.
void BleApp::makeTelemetryJson(const ControlTelemetry &t)
{
    teleJson.clear();

    for (int i = 0; i < NUM_PAIRS; ++i)
    {
        const InputSource source = current.pairInput[i].inputSource;
        const FlexSettings &fCfg = current.flex[i];
        const ServoSettings &sCfg = current.servo[i];
        const EmgSettings &eCfg = current.emg[i];

        const bool servoValid = (sCfg.servoPin != UNUSED_PIN && sCfg.servoPin != 0);
        const bool flexValid = (fCfg.flexPin != UNUSED_PIN && fCfg.flexPin != 0);
        const bool emgValid = eCfg.activePinsValid();

        bool includePair = false;
        if (source == InputSource::Emg)
            includePair = servoValid || emgValid;
        else
            includePair = servoValid || flexValid;

        // Skip pairs that are effectively unused so the payload stays smaller.
        if (!includePair)
            continue;

        char key[28];

        snprintf(key, sizeof(key), "emg_source_%d", i);
        teleJson[key] = t.emgSource[i];

        snprintf(key, sizeof(key), "servo_target_%d", i);
        teleJson[key] = t.servoTargetDeg[i];

        snprintf(key, sizeof(key), "servo_current_%d", i);
        teleJson[key] = t.servoCurrentDeg[i];

        snprintf(key, sizeof(key), "servo_speed_%d", i);
        teleJson[key] = t.servoSpeedDps[i];

        if (source == InputSource::Emg)
        {
            snprintf(key, sizeof(key), "emg_event_%d", i);
            teleJson[key] = t.emgEvent[i];

            snprintf(key, sizeof(key), "emg_action_%d", i);
            teleJson[key] = t.emgAction[i];

            snprintf(key, sizeof(key), "emg_cooldown_ms_%d", i);
            teleJson[key] = t.emgCooldownMs[i];

            snprintf(key, sizeof(key), "emg_bend_progress_%d", i);
            teleJson[key] = t.emgBendProgress[i];

            snprintf(key, sizeof(key), "emg_unfold_progress_%d", i);
            teleJson[key] = t.emgUnfoldProgress[i];

            snprintf(key, sizeof(key), "emg_ch0_%d", i);
            teleJson[key] = t.emgCh0[i];
        }
        else
        {
            snprintf(key, sizeof(key), "flex_raw_%d", i);
            teleJson[key] = t.flexRawOhm[i];

            snprintf(key, sizeof(key), "flex_filt_%d", i);
            teleJson[key] = t.flexFilteredOhm[i];
        }
    }

    teleJson["fsr_raw"] = t.fsrRawOhm;
    teleJson["fsr_filt"] = t.fsrFilteredOhm;
    teleJson["forceN"] = t.fsrForceN;

    teleJson["vibro_duty"] = t.vibroDuty;
    teleJson["vibro_mode"] = (uint8_t)t.vibroMode;

    teleJson["type"] = "tele";
}

// Queue telemetry for transmission; actual BLE sending happens from loop().
void BleApp::sendTelemetry(const ControlTelemetry &t)
{
    if (!teleEnabled)
        return;
    makeTelemetryJson(t);
    teleDirty = true;
}

// Service deferred config/ACK sends first, then emit telemetry at a throttled rate.
void BleApp::loop()
{
    updateLed();

    const uint32_t now = millis();

    if (pendingSendConfig)
    {
        pendingSendConfig = false;
        sendConfig();

        if (pendingSendAck)
        {
            uint32_t t = now + ACK_DELAY_MS;
            if (pendingAckAtMs < t)
                pendingAckAtMs = t;
        }
    }

    if (pendingSendAck && (pendingAckAtMs == 0 || now >= pendingAckAtMs))
    {
        const bool ok = pendingAckOk;

        char forTag[16] = {0};
        if (pendingAckFor[0])
            strncpy(forTag, pendingAckFor, sizeof(forTag) - 1);
        const int teleVal = pendingAckTeleVal;

        pendingSendAck = false;
        pendingAckAtMs = 0;
        setPendingAckMeta(nullptr, -1);

        sendAck(ok, (forTag[0] ? forTag : nullptr), teleVal);
    }

    if (!teleEnabled)
        return;
    if (!teleDirty)
        return;
    if (now < telePauseUntilMs)
        return;
    if (now - lastTeleSend < TELE_MIN_PERIOD_MS)
        return;

    lastTeleSend = now;
    teleDirty = false;

    sendJsonChunked(teleJson, chTele, /*pauseTele=*/false, /*useIndicate=*/false);
}
