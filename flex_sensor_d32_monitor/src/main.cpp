#include <Arduino.h>
#include <math.h>

static constexpr uint8_t FLEX_PIN = 32;
static constexpr uint32_t R_DIV_OHM = 47000;
static constexpr float VCC = 3.3f;
static constexpr int ADC_MAX = 4095;
static constexpr int ADC_SAMPLES = 16;
static constexpr uint32_t PRINT_PERIOD_MS = 200;

static int readAdcAverage(uint8_t pin, int samples)
{
    if (samples < 1)
        samples = 1;

    uint32_t sum = 0;
    for (int i = 0; i < samples; ++i)
    {
        sum += analogRead(pin);
        delayMicroseconds(120);
    }

    return (int)(sum / (uint32_t)samples);
}

static float adcToVoltage(int raw)
{
    return ((float)raw / (float)ADC_MAX) * VCC;
}

static float flexResistanceOhmFromRaw(int raw)
{
    if (raw <= 0)
        return INFINITY;

    if (raw >= ADC_MAX)
        raw = ADC_MAX - 1;

    const float vOut = adcToVoltage(raw);
    if (vOut <= 0.0001f)
        return INFINITY;

    // SparkFun divider: flex sensor to 3.3V, 47k resistor to GND, ADC on the midpoint.
    return (float)R_DIV_OHM * (VCC / vOut - 1.0f);
}

void setup()
{
    Serial.begin(115200);
    delay(400);

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    analogSetPinAttenuation(FLEX_PIN, ADC_11db);

    pinMode(FLEX_PIN, INPUT);

    // Warm up the ADC channel a bit before the first printed value.
    for (int i = 0; i < 8; ++i)
    {
        (void)analogRead(FLEX_PIN);
        delay(10);
    }

    Serial.println();
    Serial.println("ESP32 flex sensor monitor");
    Serial.println("Wiring:");
    Serial.println("  3.3V -> flex sensor -> GPIO32 -> 47k -> GND");
    Serial.println("Output format: raw, voltage, resistance");
    Serial.println();
}

void loop()
{
    static uint32_t lastPrintMs = 0;
    const uint32_t nowMs = millis();

    if (nowMs - lastPrintMs < PRINT_PERIOD_MS)
        return;

    lastPrintMs = nowMs;

    const int raw = readAdcAverage(FLEX_PIN, ADC_SAMPLES);
    const float voltage = adcToVoltage(raw);
    const float resistanceOhm = flexResistanceOhmFromRaw(raw);

    Serial.print("raw=");
    Serial.print(raw);
    Serial.print("  voltage=");
    Serial.print(voltage, 4);
    Serial.print(" V  resistance=");

    if (isfinite(resistanceOhm))
        Serial.print(resistanceOhm, 1);
    else
        Serial.print("INF");

    Serial.println(" ohm");
}

