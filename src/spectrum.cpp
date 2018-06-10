#include "common.h"
#include "spectrum.h"
#include <cassert>

static float compute_average_value_for_range(const float* lambdas, const float* values, int n, float range_start, float range_end) {
    assert(n >= 2);
    assert(range_start < range_end);

    if (range_start >= lambdas[n-1] || range_end <= lambdas[0])
        return 0.f;

    float integral = 0.f;

    range_start = std::max(range_start, lambdas[0]);
    range_end = std::min(range_end, lambdas[n-1]);

    // Get the first sample that starts contributing to the result.
    int i = 0;
    while (range_start >= lambdas[i+1]) {
        i++;
    }

    // Iterate until the last sample that contributes to the result.
    // Use piecewise linear reconstruction to compute the integral.
    for (; i < n-1 && lambdas[i] < range_end; i++) {
        float l0, v0;
        if (range_start > lambdas[i]) {
            const float t = (range_start - lambdas[i]) / (lambdas[i+1] - lambdas[i]);
            l0 = range_start;
            v0 = lerp(t, values[i], values[i+1]);
        } else {
            l0 = lambdas[i];
            v0 = values[i];
        }

        float l1, v1;
        if (range_end < lambdas[i+1]) {
            const float t = (range_end - lambdas[i]) / (lambdas[i+1] - lambdas[i]);
            l1 = range_end;
            v1 = lerp(t, values[i], values[i+1]);
        } else {
            l1 = lambdas[i+1];
            v1 = values[i+1];
        }

        integral += 0.5f*(v0 + v1) * (l1 - l0);
    }

    return integral / (range_end - range_start);
}

Sampled_Spectrum Sampled_Spectrum::from_tabulated_data(const float* lambdas, const float* values, int n) {
    Sampled_Spectrum s;

    for (int i = 0; i < Sample_Count; i++) {
        assert(i == 0 || lambdas[i] > lambdas[i-1]);

        float interval_start = Wavelength_Range_Start + Interval_Length * float(i);
        float interval_end = interval_start + Interval_Length;

        s.c[i] = compute_average_value_for_range(lambdas, values, n, interval_start, interval_end);
    }
    return s;
}
