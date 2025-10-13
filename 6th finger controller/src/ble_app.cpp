#include "ble_app.h"
#include <cstring>
#include <cmath>

void BleApp::putLEu8(uint8_t v, uint8_t *out) { out[0] = v; }
void BleApp::putLEu16(uint16_t v, uint8_t *out)
{
    out[0] = v & 0xFF;
    out[1] = (v >> 8) & 0xFF;
}
void BleApp::putLEu32(uint32_t v, uint8_t *out)
{
    out[0] = v & 0xFF;
    out[1] = (v >> 8) & 0xFF;
    out[2] = (v >> 16) & 0xFF;
    out[3] = (v >> 24) & 0xFF;
}

void BleApp::setLed(bool on)
{
    static bool inited = false;
    if (!inited)
    {
        inited = true;
        pinMode(BLE_LED_PIN, OUTPUT);
    }
    digitalWrite(BLE_LED_PIN, on ? HIGH : LOW);
}
void BleApp::updateLed()
{
    uint32_t now = millis();
    switch (ledMode)
    {
    case LedMode::Advertising:
        if (now - lastBlink >= 800)
        {
            lastBlink = now;
            setLed(true);
            delay(20);
            setLed(false);
        }
        break;
    case LedMode::Connected:
        setLed(true);
        break;
    case LedMode::Notifying:
        if (now - lastBlink >= 2000)
        {
            lastBlink = now;
            setLed(true);
            delay(15);
            setLed(false);
        }
        break;
    }
}

namespace
{
    class SCallbacks : public NimBLEServerCallbacks
    {
        BleApp *app;

    public:
        explicit SCallbacks(BleApp *a) : app(a) {}
        void onConnect(NimBLEServer *) override
        {
            if (app)
                app->onConnect();
        }
        void onDisconnect(NimBLEServer *) override
        {
            if (app)
                app->onDisconnect();
            NimBLEDevice::startAdvertising();
        }
    };
}

void BleApp::onConnect() { ledMode = LedMode::Connected; }
void BleApp::onDisconnect() { ledMode = LedMode::Advertising; }

void BleApp::loadFromNVS()
{
    nvs.begin("cfg", true);
    Settings s = curr;
    s.fsrPin = nvs.getUChar("fsrPin", s.fsrPin);
    s.fsrPullupOhm = nvs.getUShort("fsrPull", s.fsrPullupOhm);
    s.fsrStartOhm = nvs.getULong("fsrStart", s.fsrStartOhm);
    s.fsrMaxOhm = nvs.getULong("fsrMax", s.fsrMaxOhm);

    s.flexPin = nvs.getUChar("flexPin", s.flexPin);
    s.flexFlatOhm = nvs.getULong("flexFlat", s.flexFlatOhm);
    s.flexBendOhm = nvs.getULong("flexBend", s.flexBendOhm);

    s.vibroPin = nvs.getUChar("vibPin", s.vibroPin);
    s.vibroFreqHz = nvs.getUShort("vibFreq", s.vibroFreqHz);
    s.vibroThreshold = nvs.getUShort("vibThr", s.vibroThreshold);
    s.vibroPowerPct = nvs.getUChar("vibPwr", s.vibroPowerPct);

    s.servoPin = nvs.getUChar("servoPin", s.servoPin);
    s.servoMinDeg = nvs.getUChar("servoMin", s.servoMinDeg);
    s.servoMaxDeg = nvs.getUChar("servoMax", s.servoMaxDeg);
    s.servoManual = nvs.getUChar("servoMan", s.servoManual);
    s.servoManualDeg = nvs.getUChar("servoManDeg", s.servoManualDeg);
    nvs.end();
    curr = s;
}
void BleApp::saveToNVS(const Settings &s)
{
    nvs.begin("cfg", false);
    nvs.putUChar("fsrPin", s.fsrPin);
    nvs.putUShort("fsrPull", s.fsrPullupOhm);
    nvs.putULong("fsrStart", s.fsrStartOhm);
    nvs.putULong("fsrMax", s.fsrMaxOhm);

    nvs.putUChar("flexPin", s.flexPin);
    nvs.putULong("flexFlat", s.flexFlatOhm);
    nvs.putULong("flexBend", s.flexBendOhm);

    nvs.putUChar("vibPin", s.vibroPin);
    nvs.putUShort("vibFreq", s.vibroFreqHz);
    nvs.putUShort("vibThr", s.vibroThreshold);
    nvs.putUChar("vibPwr", s.vibroPowerPct);

    nvs.putUChar("servoPin", s.servoPin);
    nvs.putUChar("servoMin", s.servoMinDeg);
    nvs.putUChar("servoMax", s.servoMaxDeg);
    nvs.putUChar("servoMan", s.servoManual);
    nvs.putUChar("servoManDeg", s.servoManualDeg);
    nvs.end();
}

void BleApp::buildTelemetry()
{
    svcTele = server->createService(UUID_TELEMETRY_SVC);
    chFlex = svcTele->createCharacteristic(UUID_FLEX, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    chServo = svcTele->createCharacteristic(UUID_SERVO, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    chFsr = svcTele->createCharacteristic(UUID_FSR, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    uint8_t z[4] = {0, 0, 0, 0};
    chFlex->setValue(z, 4);
    chServo->setValue(z, 4);
    chFsr->setValue(z, 4);
    svcTele->start();
}

void BleApp::installCfgHandlers()
{
    static uint8_t b1[1], b2[2], b4[4];
    putLEu8(curr.fsrPin, b1);
    c_fsrPin->setValue(b1, 1);
    putLEu16(curr.fsrPullupOhm, b2);
    c_fsrPull->setValue(b2, 2);
    putLEu32(curr.fsrStartOhm, b4);
    c_fsrStart->setValue(b4, 4);
    putLEu32(curr.fsrMaxOhm, b4);
    c_fsrMax->setValue(b4, 4);
    putLEu8(curr.flexPin, b1);
    c_flexPin->setValue(b1, 1);
    putLEu32(curr.flexFlatOhm, b4);
    c_flexFlat->setValue(b4, 4);
    putLEu32(curr.flexBendOhm, b4);
    c_flexBend->setValue(b4, 4);
    putLEu8(curr.vibroPin, b1);
    c_vibPin->setValue(b1, 1);
    putLEu16(curr.vibroFreqHz, b2);
    c_vibFreq->setValue(b2, 2);
    putLEu16(curr.vibroThreshold, b2);
    c_vibThr->setValue(b2, 2);
    putLEu8(curr.vibroPowerPct, b1);
    c_vibPwr->setValue(b1, 1);
    putLEu8(curr.servoPin, b1);
    c_servoPin->setValue(b1, 1);
    putLEu8(curr.servoMinDeg, b1);
    c_servoMin->setValue(b1, 1);
    putLEu8(curr.servoMaxDeg, b1);
    c_servoMax->setValue(b1, 1);
    putLEu8(curr.servoManual, b1);
    c_servoMan->setValue(b1, 1);
    putLEu8(curr.servoManualDeg, b1);
    c_servoManDeg->setValue(b1, 1);

    struct WCB : public NimBLECharacteristicCallbacks
    {
        BleApp *app;
        Settings *pend;
        explicit WCB(BleApp *a, Settings *p) : app(a), pend(p) {}
        void onWrite(NimBLECharacteristic *c) override
        {
            const std::string &v = c->getValue();
            if (v.empty())
                return;
            auto u8 = [&]()
            { return static_cast<uint8_t>(v[0]); };
            auto u16 = [&]()
            { return (uint16_t)((uint8_t)v[0] | ((uint16_t)(uint8_t)v[1] << 8)); };
            auto u32 = [&]()
            {
                return (uint32_t)((uint8_t)v[0]) | ((uint32_t)(uint8_t)v[1] << 8) |
                       ((uint32_t)(uint8_t)v[2] << 16) | ((uint32_t)(uint8_t)v[3] << 24);
            };

            const char *uuid = c->getUUID().toString().c_str();
            if (!strcmp(uuid, UUID_CFG_FSR_PIN))
                pend->fsrPin = u8();
            else if (!strcmp(uuid, UUID_CFG_FSR_PULL))
                pend->fsrPullupOhm = u16();
            else if (!strcmp(uuid, UUID_CFG_FSR_START))
                pend->fsrStartOhm = u32();
            else if (!strcmp(uuid, UUID_CFG_FSR_MAX))
                pend->fsrMaxOhm = u32();
            else if (!strcmp(uuid, UUID_CFG_FLEX_PIN))
                pend->flexPin = u8();
            else if (!strcmp(uuid, UUID_CFG_FLEX_FLAT))
                pend->flexFlatOhm = u32();
            else if (!strcmp(uuid, UUID_CFG_FLEX_BEND))
                pend->flexBendOhm = u32();
            else if (!strcmp(uuid, UUID_CFG_VIB_PIN))
                pend->vibroPin = u8();
            else if (!strcmp(uuid, UUID_CFG_VIB_FREQ))
                pend->vibroFreqHz = u16();
            else if (!strcmp(uuid, UUID_CFG_VIB_THRESH))
                pend->vibroThreshold = u16();
            else if (!strcmp(uuid, UUID_CFG_VIB_PWR))
                pend->vibroPowerPct = u8();
            else if (!strcmp(uuid, UUID_CFG_SERVO_PIN))
                pend->servoPin = u8();
            else if (!strcmp(uuid, UUID_CFG_SERVO_MIN))
                pend->servoMinDeg = u8();
            else if (!strcmp(uuid, UUID_CFG_SERVO_MAX))
                pend->servoMaxDeg = u8();
            else if (!strcmp(uuid, UUID_CFG_SERVO_MAN))
                pend->servoManual = u8();
            else if (!strcmp(uuid, UUID_CFG_SERVO_MAN_D))
                pend->servoManualDeg = u8();
            else if (!strcmp(uuid, UUID_CFG_APPLY))
            {
                if (u8() == 1)
                    app->applyNow();
            }
            app->havePending = true;
        }
    };
    auto *cb = new WCB(this, &pending);
    c_fsrPin->setCallbacks(cb);
    c_fsrPull->setCallbacks(cb);
    c_fsrStart->setCallbacks(cb);
    c_fsrMax->setCallbacks(cb);
    c_flexPin->setCallbacks(cb);
    c_flexFlat->setCallbacks(cb);
    c_flexBend->setCallbacks(cb);
    c_vibPin->setCallbacks(cb);
    c_vibFreq->setCallbacks(cb);
    c_vibThr->setCallbacks(cb);
    c_vibPwr->setCallbacks(cb);
    c_servoPin->setCallbacks(cb);
    c_servoMin->setCallbacks(cb);
    c_servoMax->setCallbacks(cb);
    c_servoMan->setCallbacks(cb);
    c_servoManDeg->setCallbacks(cb);
    c_apply->setCallbacks(cb);
}

void BleApp::buildConfig()
{
    svcCfg = server->createService(UUID_CFG_SVC);
    auto RW = NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE;
    c_fsrPin = svcCfg->createCharacteristic(UUID_CFG_FSR_PIN, RW);
    c_fsrPull = svcCfg->createCharacteristic(UUID_CFG_FSR_PULL, RW);
    c_fsrStart = svcCfg->createCharacteristic(UUID_CFG_FSR_START, RW);
    c_fsrMax = svcCfg->createCharacteristic(UUID_CFG_FSR_MAX, RW);
    c_flexPin = svcCfg->createCharacteristic(UUID_CFG_FLEX_PIN, RW);
    c_flexFlat = svcCfg->createCharacteristic(UUID_CFG_FLEX_FLAT, RW);
    c_flexBend = svcCfg->createCharacteristic(UUID_CFG_FLEX_BEND, RW);
    c_vibPin = svcCfg->createCharacteristic(UUID_CFG_VIB_PIN, RW);
    c_vibFreq = svcCfg->createCharacteristic(UUID_CFG_VIB_FREQ, RW);
    c_vibThr = svcCfg->createCharacteristic(UUID_CFG_VIB_THRESH, RW);
    c_vibPwr = svcCfg->createCharacteristic(UUID_CFG_VIB_PWR, RW);
    c_servoPin = svcCfg->createCharacteristic(UUID_CFG_SERVO_PIN, RW);
    c_servoMin = svcCfg->createCharacteristic(UUID_CFG_SERVO_MIN, RW);
    c_servoMax = svcCfg->createCharacteristic(UUID_CFG_SERVO_MAX, RW);
    c_servoMan = svcCfg->createCharacteristic(UUID_CFG_SERVO_MAN, RW);
    c_servoManDeg = svcCfg->createCharacteristic(UUID_CFG_SERVO_MAN_D, RW);

    c_apply = svcCfg->createCharacteristic(UUID_CFG_APPLY, NIMBLE_PROPERTY::WRITE);

    svcCfg->start();
    installCfgHandlers();
}

void BleApp::startAdvertising()
{
    auto *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(UUID_TELEMETRY_SVC);
    adv->addServiceUUID(UUID_CFG_SVC);
    adv->setScanResponse(true);
    adv->start();
}

void BleApp::begin(const char *name)
{
    pinMode(BLE_LED_PIN, OUTPUT);
    setLed(false);
    ledMode = LedMode::Advertising;

    NimBLEDevice::init(name);
    NimBLEDevice::setPower(ESP_PWR_LVL_P7);
    NimBLEDevice::setSecurityAuth(false, false, false);

    server = NimBLEDevice::createServer();
    server->setCallbacks(new SCallbacks(this));

    loadFromNVS();
    pending = curr;

    buildTelemetry();
    buildConfig();
    startAdvertising();
}

void BleApp::applyNow()
{
    curr = pending;
    saveToNVS(curr);
    if (onReconfigure)
        onReconfigure(curr);
}

void BleApp::setTelemetry(const TelemetryValues &v)
{
    pendingTele = v;
    haveTele = true;
}

void BleApp::loop()
{
    updateLed();
    if (!haveTele)
        return;
    const uint32_t now = millis();
    bool timeOk = (now - lastNotifyMs) >= 50;

    bool fsrChanged = fabsf(pendingTele.fsr_ohm - lastSent.fsr_ohm) > 1.0f;
    bool flexChanged = fabsf(pendingTele.flex_avg_ohm - lastSent.flex_avg_ohm) > 1.0f;
    bool servoChanged = fabsf(pendingTele.servo_deg - lastSent.servo_deg) >= 1.0f;
    if (!(timeOk || fsrChanged || flexChanged || servoChanged))
        return;

    uint8_t b[4];
    memcpy(b, &pendingTele.fsr_ohm, 4);
    chFsr->setValue(b, 4);
    chFsr->notify();
    memcpy(b, &pendingTele.flex_avg_ohm, 4);
    chFlex->setValue(b, 4);
    chFlex->notify();
    memcpy(b, &pendingTele.servo_deg, 4);
    chServo->setValue(b, 4);
    chServo->notify();

    lastSent = pendingTele;
    lastNotifyMs = now;
    haveTele = false;
    ledMode = LedMode::Notifying;
}
