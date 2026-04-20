#include "ble_app.h"

static void ledSet(bool on)
{
    pinMode(BLE_LED_PIN, OUTPUT);
    digitalWrite(BLE_LED_PIN, on ? HIGH : LOW);
}

void BleApp::callNotify(NimBLECharacteristic *ch)
{
    if (!ch)
        return;
    ch->notify();
}

void BleApp::callIndicate(NimBLECharacteristic *ch)
{
    if (!ch)
        return;
    ch->indicate();
}

void BleApp::updateLed()
{
    uint32_t now = millis();

    switch (led)
    {
    case LedMode::Adv:
        if (now - lastBlink > 900)
        {
            lastBlink = now;
            ledSet(true);
            delay(25);
            ledSet(false);
        }
        break;

    case LedMode::Conn:
        ledSet(true);
        break;

    case LedMode::Notify:
        if (now - lastBlink > 1500)
        {
            lastBlink = now;
            ledSet(true);
            delay(20);
            ledSet(false);
        }
        break;
    }
}

void BleApp::loadSettings()
{
    nvs.begin("cfg", true);

    Settings s;

    String raw = nvs.getString("json", "");
    if (raw.length() > 0)
    {
        StaticJsonDocument<4096> doc;
        auto err = deserializeJson(doc, raw);
        if (!err)
        {
            s.applyJson(doc);
        }
    }
    else
    {
        s.fsrPin = nvs.getUChar("fsrPin", s.fsrPin);
        s.fsrPullupOhm = nvs.getULong("fsrPull", s.fsrPullupOhm);
        s.fsrSoftThresholdN = nvs.getFloat("fsrSoft", s.fsrSoftThresholdN);
        s.fsrHardMaxN = nvs.getFloat("fsrHard", s.fsrHardMaxN);

        s.flex[0].flexPin = nvs.getUChar("flexPin", s.flex[0].flexPin);
        s.flex[0].flexPullupOhm = nvs.getULong("flexPull", s.flex[0].flexPullupOhm);
        s.flex[0].flexStraightOhm = nvs.getULong("flxSt", s.flex[0].flexStraightOhm);
        s.flex[0].flexBendOhm = nvs.getULong("flxBd", s.flex[0].flexBendOhm);

        s.vibroPin = nvs.getUChar("vPin", s.vibroPin);
        s.vibroFreqHz = nvs.getUShort("vFreq", s.vibroFreqHz);
        s.vibroMode = (VibroMode)nvs.getUChar("vMode", (uint8_t)s.vibroMode);
        s.vibroMaxDuty = nvs.getUChar("vMax", s.vibroMaxDuty);
        s.vibroMinDuty = nvs.getUChar("vMin", s.vibroMinDuty);
        s.vibroSoftPower = nvs.getUChar("vSoft", s.vibroSoftPower);
        s.vibroPulseBase = nvs.getUChar("vBase", s.vibroPulseBase);

        s.servo[0].servoPin = nvs.getUChar("sPin", s.servo[0].servoPin);
        s.servo[0].servoMinDeg = nvs.getUChar("sMn", s.servo[0].servoMinDeg);
        s.servo[0].servoMaxDeg = nvs.getUChar("sMx", s.servo[0].servoMaxDeg);
        s.servo[0].servoManual =
            (ServoManualMode)nvs.getUChar("sMod", (uint8_t)s.servo[0].servoManual);
        s.servo[0].servoManualDeg = nvs.getUChar("sMD", s.servo[0].servoManualDeg);
        s.servo[0].servoMaxSpeedDegPerSec = nvs.getFloat("sSpd", s.servo[0].servoMaxSpeedDegPerSec);

        s.pinCode = nvs.getUShort("pin", 0);
    }

    current = s;
    nvs.end();
}

void BleApp::saveSettings()
{
    nvs.begin("cfg", false);

    StaticJsonDocument<4096> doc;
    current.toJson(doc);
    String raw;
    serializeJson(doc, raw);
    nvs.putString("json", raw);

    nvs.putUShort("pin", current.pinCode);

    nvs.end();
}

class CfgInCB : public NimBLECharacteristicCallbacks
{
public:
    BleApp *app;
    CfgInCB(BleApp *a) : app(a) {}

    void onWrite(NimBLECharacteristic *c) override
    {
        app->handleChunk(c->getValue());
    }
};

class ServoLiveCB : public NimBLECharacteristicCallbacks
{
public:
    BleApp *app;
    ServoLiveCB(BleApp *a) : app(a) {}

    void onWrite(NimBLECharacteristic *c) override
    {
        app->handleServoLive(c->getValue());
    }
};

class ConnCB : public NimBLEServerCallbacks
{
public:
    BleApp *app;
    ConnCB(BleApp *a) : app(a) {}

    void onConnect(NimBLEServer *) override
    {
        app->led = BleApp::LedMode::Conn;

        app->teleEnabled = false;

        app->pendingSendConfig = false;
        app->pendingSendAck = false;
        app->pendingAckOk = true;
        app->pendingAckAtMs = 0;

        app->setPendingAckMeta(nullptr, -1);

        app->telePauseUntilMs = millis() + 800;

        app->authed = (app->current.pinCode == 0);
    }

    void onDisconnect(NimBLEServer *) override
    {
        app->led = BleApp::LedMode::Adv;
        app->teleEnabled = false;

        app->pendingSendConfig = false;
        app->pendingSendAck = false;
        app->pendingAckOk = true;
        app->pendingAckAtMs = 0;

        app->setPendingAckMeta(nullptr, -1);

        app->authed = false;

        NimBLEDevice::startAdvertising();
    }
};

void BleApp::setupBLE()
{
    server = NimBLEDevice::createServer();
    server->setCallbacks(new ConnCB(this));

    auto *svc = server->createService("6F1A0000-0000-4A4A-AA00-001122334400");

    chCfgIn = svc->createCharacteristic(
        "6F1A0001-0000-4A4A-AA00-001122334400",
        NIMBLE_PROPERTY::WRITE);
    chCfgIn->setCallbacks(new CfgInCB(this));

    chCfgOut = svc->createCharacteristic(
        "6F1A0002-0000-4A4A-AA00-001122334400",
        NIMBLE_PROPERTY::INDICATE);

    chAck = svc->createCharacteristic(
        "6F1A0003-0000-4A4A-AA00-001122334400",
        NIMBLE_PROPERTY::INDICATE);

    chTele = svc->createCharacteristic(
        "6F1A0004-0000-4A4A-AA00-001122334400",
        NIMBLE_PROPERTY::NOTIFY);

    chServoLive = svc->createCharacteristic(
        "6F1A0005-0000-4A4A-AA00-001122334400",
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    chServoLive->setCallbacks(new ServoLiveCB(this));

    svc->start();
}

void BleApp::startAdvertising()
{
    auto *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID("6F1A0000-0000-4A4A-AA00-001122334400");
    adv->setScanResponse(true);
    adv->start();
}

void BleApp::begin(const char *name)
{
    pinMode(BLE_LED_PIN, OUTPUT);
    ledSet(false);

    NimBLEDevice::init(name);
    NimBLEDevice::setPower(ESP_PWR_LVL_P7);

    txMutex = xSemaphoreCreateMutex();

    loadSettings();
    setupBLE();
    startAdvertising();
}

void BleApp::sendConfig()
{
    if (chCfgOut == nullptr)
        return;

    StaticJsonDocument<4096> doc;
    current.toJson(doc);

    doc["type"] = "cfg";
    doc["pinSet"] = (current.pinCode != 0);
    doc["authRequired"] = (!isAuthed() && current.pinCode != 0);

    if (!isAuthed() && current.pinCode != 0)
        doc.remove("pinCode");

    sendJsonChunked(doc, chCfgOut, /*pauseTele=*/true, /*useIndicate=*/true);
}

void BleApp::sendAck(bool ok, const char *forTag, int teleEnabledVal)
{
    if (chAck == nullptr)
        return;

    StaticJsonDocument<96> doc;
    doc["type"] = "ack";
    doc["ok"] = ok;

    if (forTag && forTag[0])
        doc["for"] = forTag;

    if (teleEnabledVal >= 0)
        doc["enabled"] = (teleEnabledVal != 0);

    sendJsonChunked(doc, chAck, /*pauseTele=*/true, /*useIndicate=*/true);
}

void BleApp::sendAuthAck(bool ok)
{
    if (chAck == nullptr)
        return;

    StaticJsonDocument<96> doc;
    doc["type"] = "auth";
    doc["ok"] = ok;

    sendJsonChunked(doc, chAck, /*pauseTele=*/true, /*useIndicate=*/true);
}

void BleApp::applyIncomingJson(const JsonDocument &doc)
{
    Settings s = current;
    s.applyJson(doc);

    current = s;
    saveSettings();

    if (onSettingsChanged)
        onSettingsChanged(current);
}

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

        StaticJsonDocument<4096> doc;
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

        if (strcmp(type, "cfg_get") == 0)
        {
            pendingSendConfig = true;

            pendingAckOk = true;
            pendingSendAck = true;
            pendingAckAtMs = millis() + ACK_DELAY_MS;

            setPendingAckMeta("cfg_get", -1);
            return;
        }

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
            includePair = servoValid || emgValid || t.emgChannelCount[i] > 0;
        else
            includePair = servoValid || flexValid;

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
            snprintf(key, sizeof(key), "emg_mode_%d", i);
            teleJson[key] = t.emgMode[i];

            snprintf(key, sizeof(key), "emg_channels_%d", i);
            teleJson[key] = t.emgChannelCount[i];

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

            snprintf(key, sizeof(key), "emg_ch1_%d", i);
            teleJson[key] = t.emgCh1[i];

            snprintf(key, sizeof(key), "emg_ch2_%d", i);
            teleJson[key] = t.emgCh2[i];
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

void BleApp::sendTelemetry(const ControlTelemetry &t)
{
    if (!teleEnabled)
        return;
    makeTelemetryJson(t);
    teleDirty = true;
}

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