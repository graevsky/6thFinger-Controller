#include <Arduino.h>
#include <ESP32Servo.h>
#include "ble_app.h"

BleApp ble;
Servo servo;

static const int SAMPLES = 10;
float ring[SAMPLES];
int ridx = 0;
float sum = 0, avg = 0;
float angle = 0.f;

static inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  if (in_max == in_min)
    return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void vibroPulse(uint8_t pin, uint16_t freqHz, uint8_t powerPct, bool enabled)
{
  static unsigned long t0 = 0;
  static bool phase = false;
  if (!enabled)
  {
    digitalWrite(pin, LOW);
    phase = false;
    return;
  }
  const unsigned long now = millis();
  uint16_t period = max<uint16_t>(5, 1000u / max<uint16_t>(1, freqHz));
  uint16_t onMs = (uint16_t)((period * powerPct) / 100);
  if (now - t0 >= (phase ? onMs : (period - onMs)))
  {
    phase = !phase;
    t0 = now;
    digitalWrite(pin, phase ? HIGH : LOW);
  }
}

static float adcToOhm(uint16_t adc, float vcc, uint32_t pullup)
{
  float v = adc * (vcc / 4095.0f);
  if (v < 0.001f)
    v = 0.001f;
  return pullup * (vcc / v - 1.0f);
}

static void reconfigureHW(const Settings &s)
{
  analogReadResolution(12);
  analogSetPinAttenuation(s.fsrPin, ADC_11db);
  analogSetPinAttenuation(s.flexPin, ADC_11db);

  pinMode(s.vibroPin, OUTPUT);
  digitalWrite(s.vibroPin, LOW);

  servo.detach();
  servo.setPeriodHertz(50);
  servo.attach(s.servoPin, 500, 2500);
  angle = s.servoManual ? s.servoManualDeg : s.servoMinDeg;
  servo.write((int)angle);

  sum = 0;
  ridx = 0;
  for (int i = 0; i < SAMPLES; i++)
    ring[i] = 0;
}

void setup()
{
  Serial.begin(115200);
  ble.begin("ESP32-Flex6");
  ble.onReconfigure = reconfigureHW;
  reconfigureHW(ble.getSettings());
}

void loop()
{
  const Settings &s = ble.getSettings();

  digitalWrite(s.vibroPin, LOW);

  const float VCC = 3.3f;
  uint16_t fsrAdc = analogRead(s.fsrPin);
  float Rfsr = adcToOhm(fsrAdc, VCC, s.fsrPullupOhm);

  const float R_PULLUP_FLEX = 47000.0f;
  uint16_t flexAdc = analogRead(s.flexPin);
  float Rflex = adcToOhm(flexAdc, VCC, (uint32_t)R_PULLUP_FLEX);

  bool hardPress = (Rfsr <= (float)s.fsrMaxOhm);
  bool softPress = (Rfsr <= (float)s.fsrStartOhm) && !hardPress;
  bool fsrActive = hardPress || softPress;

  if (!fsrActive)
  {
    sum -= ring[ridx];
    ring[ridx] = Rflex;
    sum += ring[ridx];
    ridx = (ridx + 1) % SAMPLES;
    avg = sum / SAMPLES;

    float minA = s.servoMinDeg, maxA = s.servoMaxDeg;
    float target = s.servoManual ? s.servoManualDeg
                                 : constrain(fmap(avg, s.flexFlatOhm, s.flexBendOhm, minA, maxA), minA, maxA);

    if (fabsf(target - angle) > 2.0f)
    {
      angle = target;
      servo.write((int)angle);
    }
  }

  if (hardPress)
  {
    digitalWrite(s.vibroPin, HIGH);
  }
  else
  {
    vibroPulse(s.vibroPin, s.vibroFreqHz, s.vibroPowerPct, softPress);
  }

  TelemetryValues tv{};
  tv.fsr_ohm = Rfsr;
  tv.flex_avg_ohm = avg;
  tv.servo_deg = angle;
  ble.setTelemetry(tv);
  ble.loop();

  static uint32_t lastLog = 0;
  uint32_t now = millis();
  if (now - lastLog >= 100)
  {
    lastLog = now;
    Serial.printf("FSR: %.0f Ω, Flex avg: %.0f Ω, Servo: %.0f°\n", Rfsr, avg, angle);
  }

  delay(5);
}
