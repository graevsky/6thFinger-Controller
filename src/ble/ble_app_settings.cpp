// BLE settings persistence and config-response helpers.
#include "ble_app.h"

// Restore the last saved settings model from the JSON blob stored in NVS.
void BleApp::loadSettings()
{
    nvs.begin("cfg", true);

    Settings s;

    String raw = nvs.getString("json", "");
    if (raw.length() > 0)
    {
        StaticJsonDocument<BleApp::CFG_JSON_CAPACITY> doc;
        auto err = deserializeJson(doc, raw);
        if (!err)
        {
            const uint32_t storedVersion =
                doc["settingsVersion"] | 0U;
            if (storedVersion == Settings::kSettingsVersion)
                s.applyJson(doc);
        }
    }

    current = s;
    nvs.end();
}

// Persist the full settings model as JSON plus the auth pin shortcut field.
void BleApp::saveSettings()
{
    nvs.begin("cfg", false);

    StaticJsonDocument<BleApp::CFG_JSON_CAPACITY> doc;
    current.toJson(doc);
    String raw;
    serializeJson(doc, raw);
    nvs.putString("json", raw);

    nvs.putUShort("pin", current.pinCode);

    nvs.end();
}

// Send the current settings model to the app as a chunked JSON document.
void BleApp::sendConfig()
{
    if (chCfgOut == nullptr)
        return;

    StaticJsonDocument<BleApp::CFG_JSON_CAPACITY> doc;
    current.toJson(doc);

    doc["type"] = "cfg";
    doc["pinSet"] = (current.pinCode != 0);
    doc["authRequired"] = (!isAuthed() && current.pinCode != 0);

    if (!isAuthed() && current.pinCode != 0)
        doc.remove("pinCode");

    sendJsonChunked(doc, chCfgOut, /*pauseTele=*/true, /*useIndicate=*/true);
}

// Send a generic ACK/NACK payload after config or telemetry control commands.
void BleApp::sendAck(bool ok, const char *forTag, int teleEnabledVal)
{
    if (chAck == nullptr)
        return;

    StaticJsonDocument<BleApp::ACK_JSON_CAPACITY> doc;
    doc["type"] = "ack";
    doc["ok"] = ok;

    if (forTag && forTag[0])
        doc["for"] = forTag;

    if (teleEnabledVal >= 0)
        doc["enabled"] = (teleEnabledVal != 0);

    sendJsonChunked(doc, chAck, /*pauseTele=*/true, /*useIndicate=*/true);
}

// Send the dedicated auth response payload.
void BleApp::sendAuthAck(bool ok)
{
    if (chAck == nullptr)
        return;

    StaticJsonDocument<BleApp::ACK_JSON_CAPACITY> doc;
    doc["type"] = "auth";
    doc["ok"] = ok;

    sendJsonChunked(doc, chAck, /*pauseTele=*/true, /*useIndicate=*/true);
}

// Apply an already-validated JSON config update and notify the application.
void BleApp::applyIncomingJson(const JsonDocument &doc)
{
    Settings s = current;
    s.applyJson(doc);

    current = s;
    saveSettings();

    if (onSettingsChanged)
        onSettingsChanged(current);
}
