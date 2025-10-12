#include "telemetry_ble.h"

namespace
{
    class ServerCallbacks : public NimBLEServerCallbacks
    {
    public:
        explicit ServerCallbacks(bool *flag) : connectedFlag(flag) {}
        void onConnect(NimBLEServer *pServer) override
        {
            if (connectedFlag)
                *connectedFlag = true;
        }
        void onDisconnect(NimBLEServer *pServer) override
        {
            if (connectedFlag)
                *connectedFlag = false;
            NimBLEDevice::startAdvertising();
        }

    private:
        bool *connectedFlag;
    };
} // namespace

void TelemetryBLE::begin(const char *deviceName)
{
    createGatt(deviceName);
}

void TelemetryBLE::createGatt(const char *deviceName)
{
    NimBLEDevice::init(deviceName);
    NimBLEDevice::setPower(ESP_PWR_LVL_P7);
    NimBLEDevice::setSecurityAuth(false, false, false);

    server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks(&clientConnected));

    service = server->createService(SERVICE_UUID);

    chFlex = service->createCharacteristic(
        FLEX_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    chServo = service->createCharacteristic(
        SERVO_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    uint8_t zeros[4] = {0, 0, 0, 0};
    chFlex->setValue(zeros, 4);
    chServo->setValue(zeros, 4);

    service->start();

    NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
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
    static_assert(sizeof(float) == 4, "This code assumes 32-bit float");
    memcpy(out, &f, 4);
}

void TelemetryBLE::loop()
{
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
}
