// Flex-specific filtering helpers and short history utilities.
#include "control.h"

#include <math.h>

// Store one filtered flex sample in the circular history buffer.
void Control::pushFlexHistory(int idx, float v)
{
    uint8_t pos = flexHistPos[idx];
    flexHist[idx][pos] = v;

    pos++;
    if (pos >= FLEX_HISTORY)
        pos = 0;
    flexHistPos[idx] = pos;

    if (flexHistCount[idx] < FLEX_HISTORY)
        flexHistCount[idx]++;
}

namespace
{
// Tiny insertion-sort median for the fixed-size flex history window.
float medianOfSmall(const float *arr, int n)
{
    float tmp[5];
    for (int i = 0; i < n; ++i)
        tmp[i] = arr[i];

    for (int i = 1; i < n; ++i)
    {
        float key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key)
        {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }

    if (n <= 0)
        return 0.0f;
    return tmp[n / 2];
}
}

// Compute the current flex history median for outlier rejection.
float Control::medianFlexHistory(int idx) const
{
    int n = (int)flexHistCount[idx];
    if (n <= 0)
        return 0.0f;

    float tmp[5];
    for (int i = 0; i < n; ++i)
        tmp[i] = flexHist[idx][i];

    return medianOfSmall(tmp, n);
}
