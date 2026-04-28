// Firmware entry point.
#include <Arduino.h>
#include <ESP32Servo.h>

#include "config/settings.h"
#include "control/control.h"
#include "ble/ble_app.h"

BleApp ble;
Control ctrl;
// Limits how often the main loop snapshots telemetry for BLE transmission.
static uint32_t g_lastTelePushMs = 0;

static void onSettingsChanged(const Settings &s)
{
    ctrl.reconfigure(s);
}

void setup()
{
    Serial.begin(115200);
    delay(300);

    Serial.println();
    Serial.println("========== BOOT ==========");

    ble.onSettingsChanged = onSettingsChanged;

    ble.onServoLive = [](int idx, int deg, bool stop)
    {
        if (stop)
            ctrl.liveServoStop(idx);
        else
            ctrl.liveServoSet(idx, (float)deg);
    };

    ble.begin("ESP32-Flex6");

    ctrl.begin(ble.getSettings());

    Serial.println("System ready.");
}

// Run one control iteration, throttle telemetry generation, and service BLE.
void loop()
{
    ctrl.update();

    const uint32_t nowMs = millis();
    if (nowMs - g_lastTelePushMs >= 100)
    {
        g_lastTelePushMs = nowMs;
        ble.sendTelemetry(ctrl.getTelemetry());
    }

    ble.loop();
}