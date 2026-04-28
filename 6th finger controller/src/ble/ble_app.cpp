// BLE core setup.
#include "ble_app.h"

// Drive the status LED without exposing direct GPIO writes elsewhere.
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

// Blink or hold the status LED depending on advertising/connection state.
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

// GATT callback for incoming configuration chunks.
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

// GATT callback for temporary live servo override commands.
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

// BLE server connection callbacks used to reset session state.
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

// Create the GATT server, service, and all characteristics used by the app.
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

// Start BLE advertising with the controller service UUID.
void BleApp::startAdvertising()
{
    auto *adv = NimBLEDevice::getAdvertising();
    adv->addServiceUUID("6F1A0000-0000-4A4A-AA00-001122334400");
    adv->setScanResponse(true);
    adv->start();
}

// Boot BLE, restore persisted settings, and expose the GATT service.
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
