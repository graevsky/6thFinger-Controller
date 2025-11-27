#include "ble_app.h"
#include "control.h"

static void ledSet(bool on)
{
    pinMode(BLE_LED_PIN, OUTPUT);
    digitalWrite(BLE_LED_PIN, on ? HIGH : LOW);
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

    Settings s = current;

    s.fsrPin = nvs.getUChar("fsrPin", s.fsrPin);
    s.fsrPullupOhm = nvs.getULong("fsrPull", s.fsrPullupOhm);
    s.fsrSoftThresholdN = nvs.getFloat("fsrSoft", s.fsrSoftThresholdN);
    s.fsrHardMaxN = nvs.getFloat("fsrHard", s.fsrHardMaxN);

    s.flexPin = nvs.getUChar("flexPin", s.flexPin);
    s.flexPullupOhm = nvs.getULong("flexPull", s.flexPullupOhm);
    s.flexStraightOhm = nvs.getULong("flxSt", s.flexStraightOhm);
    s.flexBendOhm = nvs.getULong("flxBd", s.flexBendOhm);

    s.vibroPin = nvs.getUChar("vPin", s.vibroPin);
    s.vibroFreqHz = nvs.getUShort("vFreq", s.vibroFreqHz);
    s.vibroMode = (VibroMode)nvs.getUChar("vMode", (uint8_t)s.vibroMode);
    s.vibroMaxDuty = nvs.getUChar("vMax", s.vibroMaxDuty);
    s.vibroMinDuty = nvs.getUChar("vMin", s.vibroMinDuty);
    s.vibroSoftPower = nvs.getUChar("vSoft", s.vibroSoftPower);
    s.vibroPulseBase = nvs.getUChar("vBase", s.vibroPulseBase);

    s.servoPin = nvs.getUChar("sPin", s.servoPin);
    s.servoMinDeg = nvs.getUChar("sMn", s.servoMinDeg);
    s.servoMaxDeg = nvs.getUChar("sMx", s.servoMaxDeg);
    s.servoManual = (ServoManualMode)nvs.getUChar("sMod", (uint8_t)s.servoManual);
    s.servoManualDeg = nvs.getUChar("sMD", s.servoManualDeg);
    s.servoMaxSpeedDegPerSec = nvs.getFloat("sSpd", s.servoMaxSpeedDegPerSec);

    current = s;
    nvs.end();
}

void BleApp::saveSettings()
{
    nvs.begin("cfg", false);

    nvs.putUChar("fsrPin", current.fsrPin);
    nvs.putULong("fsrPull", current.fsrPullupOhm);
    nvs.putFloat("fsrSoft", current.fsrSoftThresholdN);
    nvs.putFloat("fsrHard", current.fsrHardMaxN);

    nvs.putUChar("flexPin", current.flexPin);
    nvs.putULong("flexPull", current.flexPullupOhm);
    nvs.putULong("flxSt", current.flexStraightOhm);
    nvs.putULong("flxBd", current.flexBendOhm);

    nvs.putUChar("vPin", current.vibroPin);
    nvs.putUShort("vFreq", current.vibroFreqHz);
    nvs.putUChar("vMode", (uint8_t)current.vibroMode);
    nvs.putUChar("vMax", current.vibroMaxDuty);
    nvs.putUChar("vMin", current.vibroMinDuty);
    nvs.putUChar("vSoft", current.vibroSoftPower);
    nvs.putUChar("vBase", current.vibroPulseBase);

    nvs.putUChar("sPin", current.servoPin);
    nvs.putUChar("sMn", current.servoMinDeg);
    nvs.putUChar("sMx", current.servoMaxDeg);
    nvs.putUChar("sMod", (uint8_t)current.servoManual);
    nvs.putUChar("sMD", current.servoManualDeg);
    nvs.putFloat("sSpd", current.servoMaxSpeedDegPerSec);

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

class ConnCB : public NimBLEServerCallbacks
{
public:
    BleApp *app;
    ConnCB(BleApp *a) : app(a) {}

    void onConnect(NimBLEServer *) override { app->led = BleApp::LedMode::Conn; }
    void onDisconnect(NimBLEServer *) override
    {
        app->led = BleApp::LedMode::Adv;
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
        NIMBLE_PROPERTY::NOTIFY);

    chAck = svc->createCharacteristic(
        "6F1A0003-0000-4A4A-AA00-001122334400",
        NIMBLE_PROPERTY::NOTIFY);

    chTele = svc->createCharacteristic(
        "6F1A0004-0000-4A4A-AA00-001122334400",
        NIMBLE_PROPERTY::NOTIFY);

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

    loadSettings();
    setupBLE();
    startAdvertising();
}

void BleApp::applyIncomingJson(const JsonDocument &doc)
{
    Settings s = current;
    s.applyJson(doc);

    current = s;
    saveSettings();

    if (onSettingsChanged)
        onSettingsChanged(current);

    sendAck(true);

    StaticJsonDocument<512> reply;
    current.toJson(reply);
    sendJsonChunked(reply, chCfgOut);
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

        StaticJsonDocument<1024> doc;
        auto err = deserializeJson(doc, chunkBuffer);
        if (err)
        {
            sendAck(false);
            return;
        }

        applyIncomingJson(doc);
        return;
    }

    if (receivingChunks)
        chunkBuffer += s.c_str();
}

void BleApp::sendAck(bool ok)
{
    StaticJsonDocument<64> doc;
    doc["ack"] = ok;

    String s;
    serializeJson(doc, s);

    chAck->setValue(s.c_str());
    chAck->notify();
    led = LedMode::Notify;
}

void BleApp::sendJsonChunked(const JsonDocument &doc, NimBLECharacteristic *ch)
{
    String payload;
    serializeJson(doc, payload);

    const int CH = 100;

    ch->setValue("[BEGIN]");
    ch->notify();

    for (int i = 0; i < payload.length(); i += CH)
    {
        ch->setValue(payload.substring(i, i + CH).c_str());
        ch->notify();
    }

    ch->setValue("[END]");
    ch->notify();

    led = LedMode::Notify;
}

void BleApp::makeTelemetryJson(const ControlTelemetry &t)
{
    teleJson.clear();
    teleJson["flex_raw"] = t.flexRawOhm;
    teleJson["flex_filt"] = t.flexFilteredOhm;
    teleJson["fsr_raw"] = t.fsrRawOhm;
    teleJson["fsr_filt"] = t.fsrFilteredOhm;
    teleJson["forceN"] = t.fsrForceN;

    teleJson["servo_target"] = t.servoTargetDeg;
    teleJson["servo_current"] = t.servoCurrentDeg;
    teleJson["servo_speed"] = t.servoSpeedDps;

    teleJson["vibro_duty"] = t.vibroDuty;
    teleJson["vibro_mode"] = (uint8_t)t.vibroMode;
}


void BleApp::sendTelemetry(const ControlTelemetry &t)
{
    makeTelemetryJson(t);
    teleDirty = true;
}

void BleApp::loop()
{
    updateLed();

    if (teleDirty)
    {
        uint32_t now = millis();
        if (now - lastTeleSend >= 50)
        {
            lastTeleSend = now;
            teleDirty = false;
            sendJsonChunked(teleJson, chTele);
        }
    }
}
