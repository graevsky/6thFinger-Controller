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
    ble.begin("ESP32-Flex6");

    ctrl.begin(ble.getSettings());

    Serial.println("System ready.");
}

void loop()
{

    ctrl.update();
    {
        const ControlTelemetry &t = ctrl.getTelemetry();

        Serial.printf(
            "FLEX raw: %.0f Ω | filt: %.0f Ω | "
            "FSR raw: %.0f Ω | filt: %.0f Ω | Force: %.2f N | "
            "Servo: tgt %.1f° cur %.1f° spd %.1f°/s | "
            "Vib: duty %u mode %u\n",
            t.flexRawOhm,
            t.flexFilteredOhm,
            t.fsrRawOhm,
            t.fsrFilteredOhm,
            t.fsrForceN,
            t.servoTargetDeg,
            t.servoCurrentDeg,
            t.servoSpeedDps,
            t.vibroDuty,
            (uint8_t)t.vibroMode);
    }

    ble.sendTelemetry(ctrl.getTelemetry());

    ble.loop();

    delay(50);
}
