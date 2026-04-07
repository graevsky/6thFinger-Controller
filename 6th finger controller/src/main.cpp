#include <Arduino.h>
#include <ESP32Servo.h>

#include "settings.h"
#include "control.h"
#include "ble_app.h"

BleApp ble;
Control ctrl;

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

void loop()
{
    ctrl.update();

    ble.sendTelemetry(ctrl.getTelemetry());
    ble.loop();

    delay(50);
}