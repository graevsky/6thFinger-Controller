#include <Arduino.h>
#include <ESP32Servo.h>

const int fsrPin = 34;   // D34, датчик давления
const int flexPin = 35;  // D35, датчик изгиба
const int motorPin = 25; // D25, вибромотор
const int servoPin = 18; // D18, сервопривод

// Параметры FSR
int fsrReading;
float fsrVoltage;
float fsrResistance;
float fsrConductance;
float fsrForce;

const float VCC_MV = 3300.0f;
const float R_PULL = 4700.0f;

const float VCC = 3.3f;
const float R_DIV = 47000.0f; // pullup 47 кОм
const float flatResistance = 45000.0f;
const float bendResistance = 33400.0f;

// Серва
Servo myServo;
const int minAngle = 40;
const int maxAngle = 180;

// Сглаживание
float angle = 0;
const int numReadings = 10;
float readings[numReadings];
int readIndex = 0;
float total = 0;
float average = 0;

unsigned long lastPulseTime = 0;
bool motorState = false;

static inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  if (in_max == in_min)
    return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(fsrPin, ADC_11db);
  analogSetPinAttenuation(flexPin, ADC_11db);

  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW);

  myServo.setPeriodHertz(50);
  myServo.attach(servoPin, 500, 2500);

  for (int i = 0; i < numReadings; i++)
  {
    readings[i] = 0;
  }

  angle = minAngle;
  myServo.write(angle);
}

void loop()
{
  fsrReading = analogRead(fsrPin);
  fsrVoltage = (fsrReading * VCC_MV) / 4095.0f;

  if (fsrVoltage <= 1.0f)
  {
    fsrForce = 0;
  }
  else
  {
    fsrResistance = (VCC_MV - fsrVoltage) * R_PULL / fsrVoltage;
    fsrConductance = 1000000.0f / fsrResistance;

    if (fsrConductance <= 1000.0f)
    {
      fsrForce = fsrConductance / 80.0f;
    }
    else
    {
      fsrForce = (fsrConductance - 1000.0f) / 30.0f;
    }
  }

  if (fsrForce < 0)
    fsrForce = 0;
  if (fsrForce > 450)
    fsrForce = 450;

  if (fsrForce > 0)
  {
    int pulseInterval = (int)fmap(fsrForce, 0, 450, 1000, 50);
    unsigned long now = millis();

    if (!motorState && (now - lastPulseTime > (unsigned long)pulseInterval))
    {
      digitalWrite(motorPin, HIGH);
      motorState = true;
      lastPulseTime = now;
    }

    if (motorState && (now - lastPulseTime > 100UL))
    {
      digitalWrite(motorPin, LOW);
      motorState = false;
    }

    Serial.print("FSR Force (N): ");
    Serial.print(fsrForce, 1);
    Serial.print(" , Interval: ");
    Serial.println(pulseInterval);
  }
  else
  {
    digitalWrite(motorPin, LOW);
    motorState = false;
  }

  int ADCflex = analogRead(flexPin);
  float Vflex = ADCflex * VCC / 4095.0f;

  float Rflex = R_DIV * (VCC / Vflex - 1.0f);

  total -= readings[readIndex];
  readings[readIndex] = Rflex;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  average = total / numReadings;

  float newAngle = fmap(average, flatResistance, bendResistance, minAngle, maxAngle);
  newAngle = constrain(newAngle, (float)minAngle, (float)maxAngle);

  if (fabsf(newAngle - angle) > 2.0f)
  {
    angle = newAngle;
    myServo.write((int)angle);

    Serial.print("Flex R: ");
    Serial.print(average, 0);
    Serial.print(" ohm , Servo: ");
    Serial.print(angle, 0);
    Serial.println(" deg");
  }

  delay(10);
}
