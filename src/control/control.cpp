// Base control helpers: flex mock support, stable ADC reads, and sensor-domain
// conversions shared by flex, FSR, and EMG sampling.
#include "control.h"

#include <math.h>

// ================= FLEX ADC MOCK =================
// Development-only flex mock. It emulates one flex sensor by synthesizing a
// resistance curve and then converting it back to the ADC domain.
#define ENABLE_FLEX_ADC_MOCK 0

#if ENABLE_FLEX_ADC_MOCK
static constexpr uint8_t MOCK_FLEX_PIN = 32;
static constexpr float MOCK_R_FLAT_OHM = 30000.0f;
static constexpr float MOCK_R_BENT_OHM = 100000.0f;
static constexpr uint32_t MOCK_T_FLAT_MS = 5000;
static constexpr uint32_t MOCK_T_RAMP_MS = 3000;
static constexpr uint32_t MOCK_T_BENT_MS = 5000;
static constexpr uint32_t MOCK_T_BACK_MS = 3000;
static uint32_t g_mockPullOhm = 47000;

// Generate a smooth resistance profile that alternates between straight and bent.
static float mockFlexResistanceOhm()
{
    const uint32_t period =
        MOCK_T_FLAT_MS + MOCK_T_RAMP_MS + MOCK_T_BENT_MS + MOCK_T_BACK_MS;

    uint32_t t = millis() % period;

    if (t < MOCK_T_FLAT_MS)
        return MOCK_R_FLAT_OHM;
    t -= MOCK_T_FLAT_MS;

    if (t < MOCK_T_RAMP_MS)
    {
        float k = (float)t / (float)MOCK_T_RAMP_MS;
        return MOCK_R_FLAT_OHM + (MOCK_R_BENT_OHM - MOCK_R_FLAT_OHM) * k;
    }
    t -= MOCK_T_RAMP_MS;

    if (t < MOCK_T_BENT_MS)
        return MOCK_R_BENT_OHM;
    t -= MOCK_T_BENT_MS;

    float k = (float)t / (float)MOCK_T_BACK_MS;
    return MOCK_R_BENT_OHM + (MOCK_R_FLAT_OHM - MOCK_R_BENT_OHM) * k;
}

// Convert the synthetic resistance into the same ADC range as the real divider.
static int mockFlexAdcRaw()
{
    float R = mockFlexResistanceOhm();
    float pull = (float)g_mockPullOhm;

    if (pull < 1.0f)
        pull = 47000.0f;

    float ratio = pull / (pull + R);
    int raw = (int)lroundf(ratio * 4095.0f);

    if (raw < 0)
        raw = 0;
    if (raw > 4095)
        raw = 4095;
    return raw;
}

// Replace analogRead only for the configured mock flex pin.
static int analogReadMock(uint8_t pin)
{
    if (pin == MOCK_FLEX_PIN)
        return mockFlexAdcRaw();

    return (int)::analogRead(pin);
}

#define analogRead(pin) analogReadMock((uint8_t)(pin))
#endif
// ================= /FLEX ADC MOCK =================

namespace
{
    // Small float clamp helper used by the conversion routines in this file.
    float clampf(float v, float a, float b)
    {
        if (v < a)
            return a;
        if (v > b)
            return b;
        return v;
    }
}

// Bridge Control::readAdcAvgStable into the function-pointer API expected by EMG.
int Control::adcReadBridge(void *ctx, uint8_t pin, int samples)
{
    if (ctx == nullptr)
        return 0;
    return ((Control *)ctx)->readAdcAvgStable(pin, samples);
}

// Treat 0 and UNUSED_PIN as disconnected pins throughout the control stack.
bool Control::isValidPin(uint8_t pin) const
{
    return pin != 0 && pin != UNUSED_PIN;
}

// Shared first-order smoothing used for flex and FSR filtering.
float Control::smooth(float prev, float cur, float alpha)
{
    return prev * (1.0f - alpha) + cur * alpha;
}

// Read one ADC pin with extra dummy reads and settle time after channel changes.
int Control::readAdcAvgStable(uint8_t pin, int samples)
{
    if (samples < 1)
        samples = 1;

    if (samples == 1)
    {
        lastAdcPin = pin;
        return analogRead(pin);
    }

    if (pin != lastAdcPin)
    {
        for (int i = 0; i < ADC_DUMMY_READS; ++i)
        {
            (void)analogRead(pin);
            delayMicroseconds(ADC_SETTLE_US);
        }
        lastAdcPin = pin;
    }

    uint32_t sum = 0;
    for (int i = 0; i < samples; ++i)
    {
        sum += analogRead(pin);
        delayMicroseconds(110);
    }

    return (int)(sum / (uint32_t)samples);
}

// Read one resistor-divider sensor and convert the stable ADC value to ohms.
float Control::readResistanceStable(uint8_t pin, uint32_t pullupOhm, int samples)
{
    const float VCC = 3.3f;

    int raw = readAdcAvgStable(pin, samples);

    if (raw < 1)
        raw = 1;
    if (raw > 4094)
        raw = 4094;

    float v = (raw / 4095.0f) * VCC;

    if (v < 0.001f)
        return INFINITY;

    return pullupOhm * (VCC / v - 1.0f);
}

// Convert an ADC code from a divider-based sensor into resistance.
float Control::resistanceFromAdc(float adc, uint32_t pullupOhm) const
{
    const float VCC = 3.3f;

    if (pullupOhm == 0)
        return INFINITY;

    if (adc <= FSR_OPEN_ADC)
        return INFINITY;

    float v = (adc / 4095.0f) * VCC;

    if (v <= 0.005f)
        return INFINITY;

    return pullupOhm * (VCC / v - 1.0f);
}

// Clamp resistance values so telemetry stays finite and mobile-safe.
float Control::sanitizeResistanceForTelemetry(float resistanceOhm) const
{
    if (!isfinite(resistanceOhm) || resistanceOhm > FSR_MAX_REPORT_OHM)
        return FSR_MAX_REPORT_OHM;

    if (resistanceOhm < 0.0f)
        return 0.0f;

    return resistanceOhm;
}

// Map the FSR resistance curve to an approximate force in newtons.
float Control::fsrToNewton(float resistanceOhm)
{
    if (!isfinite(resistanceOhm))
        return 0.0f;

    const float R_NO_TOUCH = 150000.0f;
    const float R_FULL_SCALE = 1200.0f;

    if (resistanceOhm >= R_NO_TOUCH)
        return 0.0f;

    if (resistanceOhm <= R_FULL_SCALE)
        return current.fsrHardMaxN;

    float x = (logf(R_NO_TOUCH) - logf(resistanceOhm)) /
              (logf(R_NO_TOUCH) - logf(R_FULL_SCALE));

    x = clampf(x, 0.0f, 1.0f);
    x = powf(x, 2.2f);

    return x * current.fsrHardMaxN;
}

// Map filtered flex resistance into the configured servo angle range.
float Control::flexToAngle(float rohm, int idx) const
{
    const FlexSettings &fCfg = current.flex[idx];
    const ServoSettings &sCfg = current.servo[idx];

    const float minA = sCfg.servoMinDeg;
    const float maxA = sCfg.servoMaxDeg;

    const float st = (float)fCfg.flexStraightOhm;
    const float bd = (float)fCfg.flexBendOhm;

    if (bd <= st + 1.0f)
    {
        return (rohm <= st) ? maxA : minA;
    }

    if (rohm <= st)
        return maxA;

    if (rohm >= bd)
        return minA;

    float t = (rohm - st) / (bd - st);
    t = clampf(t, 0.0f, 1.0f);

    return maxA - t * (maxA - minA);
}

// Read, sanitize, filter, and convert one flex input into a servo target.
void Control::updateFlexInput(int idx)
{
    if (idx < 0 || idx >= NUM_PAIRS)
        return;

    const FlexSettings &fCfg = current.flex[idx];

#if ENABLE_FLEX_ADC_MOCK
    const bool mockActive = (fCfg.flexPin == MOCK_FLEX_PIN);
    if (mockActive && fCfg.flexPullupOhm > 0)
        g_mockPullOhm = fCfg.flexPullupOhm;
#else
    const bool mockActive = false;
#endif

    float rNew = readResistanceStable(fCfg.flexPin, fCfg.flexPullupOhm, FLEX_SAMPLES);

    if (!mockActive)
    {
        const float st = (float)fCfg.flexStraightOhm;
        const float bd = (float)fCfg.flexBendOhm;
        const float lo = fminf(st, bd) * 0.40f;
        const float hi = fmaxf(st, bd) * 2.50f;

        // Reject obviously broken samples so EMI or ADC glitches do not jerk the servo.
        if (!isfinite(rNew) || rNew < lo || rNew > hi)
        {
            rNew = flexStableInit[idx] ? flexStableOhm[idx] : (st > 1.0f ? st : 50000.0f);
        }
    }

    float rSample = rNew;
    const int histN = (int)flexHistCount[idx];

    // The tiny rolling median is only used as a guardrail against isolated spikes.
    if (histN >= 3)
    {
        float med = medianFlexHistory(idx);
        if (med > 1.0f)
        {
            bool outlier = (rNew > med * 2.0f) || (rNew < med * 0.5f);
            if (outlier)
            {
                flexOutlierStrikes[idx]++;
                if (flexOutlierStrikes[idx] < 2)
                {
                    rSample = med;
                }
                else
                {
                    flexOutlierStrikes[idx] = 0;
                    rSample = rNew;
                    pushFlexHistory(idx, rSample);
                }
            }
            else
            {
                flexOutlierStrikes[idx] = 0;
                rSample = rNew;
                pushFlexHistory(idx, rSample);
            }
        }
        else
        {
            flexOutlierStrikes[idx] = 0;
            rSample = rNew;
            pushFlexHistory(idx, rSample);
        }
    }
    else
    {
        flexOutlierStrikes[idx] = 0;
        rSample = rNew;
        pushFlexHistory(idx, rSample);
    }

    int pct = (int)fCfg.flexTolerancePct;
    if (pct < 1)
        pct = 1;
    if (pct > 50)
        pct = 50;

    if (!flexStableInit[idx])
    {
        flexStableInit[idx] = true;
        flexStableOhm[idx] = rSample;
    }

    float stable = flexStableOhm[idx];
    float tol = fabsf(stable) * (pct / 100.0f);
    if (tol < 1.0f)
        tol = 1.0f;

    float diff = rSample - stable;

    if (fabsf(diff) > tol)
    {
        float sign = (diff >= 0.0f) ? 1.0f : -1.0f;
        float target = stable + (diff - sign * tol);

        float absMove = fabsf(target - stable);

        float a = 0.20f;
        if (absMove > 10.0f * tol)
            a = 0.45f;
        else if (absMove > 4.0f * tol)
            a = 0.30f;

        flexStableOhm[idx] = smooth(stable, target, a);
    }

    flexFiltered[idx] = flexStableOhm[idx];
    flexRaw[idx] = rNew;

    tele.flexRawOhm[idx] = sanitizeResistanceForTelemetry(flexRaw[idx]);
    tele.flexFilteredOhm[idx] = sanitizeResistanceForTelemetry(flexFiltered[idx]);

    targetAngleDeg[idx] = flexToAngle(flexFiltered[idx], idx);
}
