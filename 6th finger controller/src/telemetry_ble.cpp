#include "telemetry_ble.h"

namespace
{
    class ServerCallbacks : public NimBLEServerCallbacks
    {
    public:
        explicit ServerCallbacks(TelemetryBLE *owner) : owner_(owner) {}
        void onConnect(NimBLEServer *) override
        {
            if (owner_)
                owner_->onConnect();
        }
        void onDisconnect(NimBLEServer *) override
        {
            if (owner_)
                owner_->onDisconnect();
            NimBLEDevice::startAdvertising();
        }

    private:
        TelemetryBLE *owner_;
    };
} // namespace

void TelemetryBLE::setLed(bool on)
{
    static bool inited = false;
    if (!inited)
    {
        inited = true;
        pinMode(BLE_LED_PIN, OUTPUT);
    }
    digitalWrite(BLE_LED_PIN, on ? HIGH : LOW);
}

void TelemetryBLE::updateLed()
{
    const uint32_t now = millis();
    switch (ledMode)
    {
    case LedMode::Advertising:
        if (now - lastBlink >= 800)
        {
            lastBlink = now;
            setLed(true);
            delay(30);
            setLed(false);
        }
        break;
    case LedMode::Connected:
        setLed(true);
        break;
    case LedMode::Notifying:
        if (now - lastBlink >= 200)
        {
            lastBlink = now;
            setLed(true);
            delay(15);
            setLed(false);
        }
        break;
    }
}

void TelemetryBLE::begin(const char *deviceName) { createGatt(deviceName); }

void TelemetryBLE::createGatt(const char *deviceName)
{
    pinMode(BLE_LED_PIN, OUTPUT);
    setLed(false);
    ledMode = LedMode::Advertising;

    NimBLEDevice::init(deviceName);
    NimBLEDevice::setPower(ESP_PWR_LVL_P7);
    NimBLEDevice::setSecurityAuth(false, false, false);

    server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks(this));

    service = server->createService(SERVICE_UUID);

    chFlex = service->createCharacteristic(
        FLEX_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    chServo = service->createCharacteristic(
        SERVO_UUID, NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    uint8_t zeros[4] = {0, 0, 0, 0};
    chFlex->setValue(zeros, 4);
    chServo->setValue(zeros, 4);

    service->start();

    auto *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID(SERVICE_UUID);
    adv->setScanResponse(true);
    adv->start();
}

bool TelemetryBLE::hasSubscribers() const
{
    return (chFlex && chFlex->getSubscribedCount() > 0) ||
           (chServo && chServo->getSubscribedCount() > 0);
}

void TelemetryBLE::setValues(const TelemetryValues &v)
{
    pending = v;
    havePending = true;
}

void TelemetryBLE::floatToLE(float f, uint8_t out[4])
{
    memcpy(out, &f, 4);
}

void TelemetryBLE::onConnect()
{
    clientConnected = true;
    ledMode = LedMode::Connected;
}
void TelemetryBLE::onDisconnect()
{
    clientConnected = false;
    ledMode = LedMode::Advertising;
}

void TelemetryBLE::loop()
{
    updateLed();

    if (!havePending)
        return;
    const uint32_t now = millis();
    const bool timeOk = (now - lastNotifyMs) >= notifyIntervalMs;

    const bool flexChanged = fabsf(pending.flex_avg_ohm - lastSent.flex_avg_ohm) > 1.0f;
    const bool servoChanged = fabsf(pending.servo_deg - lastSent.servo_deg) >= 1.0f;
    const bool needSend = timeOk || flexChanged || servoChanged;

    if (!needSend)
        return;

    if (!hasSubscribers())
    {
        uint8_t buf[4];
        floatToLE(pending.flex_avg_ohm, buf);
        chFlex->setValue(buf, 4);
        floatToLE(pending.servo_deg, buf);
        chServo->setValue(buf, 4);
        lastSent = pending;
        lastNotifyMs = now;
        havePending = false;
        ledMode = clientConnected ? LedMode::Connected : LedMode::Advertising;
        return;
    }

    uint8_t b[4];
    floatToLE(pending.flex_avg_ohm, b);
    chFlex->setValue(b, 4);
    chFlex->notify();
    floatToLE(pending.servo_deg, b);
    chServo->setValue(b, 4);
    chServo->notify();

    lastSent = pending;
    lastNotifyMs = now;
    havePending = false;
    ledMode = LedMode::Notifying;
}
