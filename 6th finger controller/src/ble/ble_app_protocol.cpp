// BLE command parsing and chunked JSON transport helpers.
#include "ble_app.h"

// Parse a live servo command from the app and forward it to the control layer.
void BleApp::handleServoLive(const std::string &s)
{
    if (!isAuthed() && current.pinCode != 0)
        return;

    if (s.empty())
        return;

    int idx = -1;
    int deg = -1;

    const char *c = s.c_str();

    if (sscanf(c, "A,%d,%d", &idx, &deg) == 2)
    {
        if (idx < 0 || idx >= NUM_PAIRS)
            return;
        if (deg < 0)
            deg = 0;
        if (deg > 180)
            deg = 180;

        if (onServoLive)
            onServoLive(idx, deg, false);

        return;
    }

    if (sscanf(c, "S,%d", &idx) == 1)
    {
        if (idx < 0 || idx >= NUM_PAIRS)
            return;

        if (onServoLive)
            onServoLive(idx, 0, true);

        return;
    }
}

// Receive chunked JSON config messages and dispatch their command types.
void BleApp::handleChunk(const std::string &s)
{
    if (s == "[BEGIN]")
    {
        receivingChunks = true;
        chunkBuffer = "";
        return;
    }

    if (s == "[END]")
    {
        receivingChunks = false;

        String json = chunkBuffer;
        chunkBuffer = "";

        StaticJsonDocument<BleApp::CFG_JSON_CAPACITY> doc;
        auto err = deserializeJson(doc, json);
        if (err)
        {
            pendingAckOk = false;
            pendingSendAck = true;
            pendingAckAtMs = millis();

            setPendingAckMeta("cfg_parse", -1);
            return;
        }

        const char *type = doc["type"] | "cfg_set";

        // Auth is handled before generic config changes.
        if (strcmp(type, "auth") == 0)
        {
            if (current.pinCode == 0)
            {
                authed = true;
                sendAuthAck(true);
                pendingSendConfig = true;
                return;
            }

            const char *pinStr = doc["pin"] | "";
            bool ok = false;

            if (strlen(pinStr) == 4)
            {
                bool digits = true;
                for (int i = 0; i < 4; i++)
                {
                    if (pinStr[i] < '0' || pinStr[i] > '9')
                    {
                        digits = false;
                        break;
                    }
                }
                if (digits)
                {
                    int code = atoi(pinStr);
                    ok = (code == (int)current.pinCode);
                }
            }

            if (ok)
            {
                authed = true;
                sendAuthAck(true);
                pendingSendConfig = true;
            }
            else
            {
                sendAuthAck(false);
            }
            return;
        }

        // The app can acknowledge config receipt explicitly.
        if (strcmp(type, "cfg_ok") == 0)
        {
            if (!isAuthed() && current.pinCode != 0)
            {
                sendAuthAck(false);
                return;
            }

            pendingAckOk = true;
            pendingSendAck = true;
            pendingAckAtMs = millis();

            setPendingAckMeta("cfg_ok", -1);
            return;
        }

        // Request the current config snapshot from firmware.
        if (strcmp(type, "cfg_get") == 0)
        {
            pendingSendConfig = true;

            pendingAckOk = true;
            pendingSendAck = true;
            pendingAckAtMs = millis() + ACK_DELAY_MS;

            setPendingAckMeta("cfg_get", -1);
            return;
        }

        // Reboot is handled here so the ACK can be emitted before restart.
        if (strcmp(type, "reboot") == 0)
        {
            if (!isAuthed() && current.pinCode != 0)
            {
                pendingAckOk = false;
                pendingSendAck = true;
                pendingAckAtMs = millis() + ACK_DELAY_MS;
                setPendingAckMeta("reboot", -1);
                return;
            }

            sendAck(true, "reboot", -1);
            delay(250);
            ESP.restart();
            return;
        }

        // Telemetry enable/disable is separate from settings persistence.
        if (strcmp(type, "tele_set") == 0)
        {
            if (!isAuthed() && current.pinCode != 0)
            {
                pendingAckOk = false;
                pendingSendAck = true;
                pendingAckAtMs = millis() + ACK_DELAY_MS;
                setPendingAckMeta("tele_set", -1);
                return;
            }

            bool en = doc["enabled"] | true;
            teleEnabled = en;

            pendingAckOk = true;
            pendingSendAck = true;
            pendingAckAtMs = millis() + ACK_DELAY_MS;

            setPendingAckMeta("tele_set", en ? 1 : 0);
            return;
        }

        if (!isAuthed() && current.pinCode != 0)
        {
            pendingAckOk = false;
            pendingSendAck = true;
            pendingAckAtMs = millis() + ACK_DELAY_MS;

            setPendingAckMeta("auth", -1);
            return;
        }

        applyIncomingJson(doc);

        pendingSendConfig = true;

        pendingAckOk = true;
        pendingSendAck = true;
        pendingAckAtMs = millis() + ACK_DELAY_MS;

        setPendingAckMeta("cfg_set", -1);
        return;
    }

    if (receivingChunks)
        chunkBuffer += s.c_str();
}

// Send a JSON document over BLE in 20-byte chunks.
void BleApp::sendJsonChunked(
    const JsonDocument &doc,
    NimBLECharacteristic *ch,
    bool pauseTele,
    bool useIndicate)
{
    if (ch == nullptr)
        return;

    if (pauseTele)
        telePauseUntilMs = millis() + PAUSE_TELE_MS;

    if (txMutex != nullptr)
    {
        if (xSemaphoreTake(txMutex, pdMS_TO_TICKS(4000)) != pdTRUE)
            return;
    }

    String payload;
    serializeJson(doc, payload);

    const int CHUNK_BYTES = 20;
    const int dly = useIndicate ? 55 : (pauseTele ? 8 : 2);

    auto sendPart = [&](const String &s)
    {
        ch->setValue(s);

        if (useIndicate)
            callIndicate(ch);
        else
            callNotify(ch);

        delay(dly);
        yield();
    };

    sendPart("[BEGIN]");

    for (int i = 0; i < (int)payload.length(); i += CHUNK_BYTES)
        sendPart(payload.substring(i, i + CHUNK_BYTES));

    sendPart("[END]");

    led = LedMode::Notify;

    if (txMutex != nullptr)
        xSemaphoreGive(txMutex);
}
