#include "std.h"
#include "common.h"
#include "spectrum.h"

#include "colorimetry.h"
#include "math.h"

static float compute_average_value_for_range(const float* lambdas, const float* values, int n, float range_start, float range_end) {
    ASSERT(n >= 2);
    ASSERT(range_start < range_end);

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
            v0 = lerp(values[i], values[i+1], t);
        } else {
            l0 = lambdas[i];
            v0 = values[i];
        }

        float l1, v1;
        if (range_end < lambdas[i+1]) {
            const float t = (range_end - lambdas[i]) / (lambdas[i+1] - lambdas[i]);
            l1 = range_end;
            v1 = lerp(values[i], values[i+1], t);
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
        float interval_start = Wavelength_Range_Start + Interval_Length * float(i);
        float interval_end = interval_start + Interval_Length;

        s.c[i] = compute_average_value_for_range(lambdas, values, n, interval_start, interval_end);
    }
    return s;
}

Sampled_Spectrum Sampled_Spectrum::constant_spectrum(float c) {
    Sampled_Spectrum s;
    for (int i = 0; i < Sample_Count; i++) {
        s.c[i] = c;
    }
    return s;
}

Vector3 Sampled_Spectrum::emission_spectrum_to_XYZ() const {
    Vector3 xyz{0.f};
    for (int i = 0; i < Sample_Count; i++) {
        xyz[0] += c[i] * CIE_X.c[i];
        xyz[1] += c[i] * CIE_Y.c[i];
        xyz[2] += c[i] * CIE_Z.c[i];
    }
    xyz *= (float)Interval_Length;
    return xyz;
}

Vector3 Sampled_Spectrum::reflectance_spectrum_to_XYZ_for_D65_illuminant() const {
    Vector3 xyz{ 0.f };
    for (int i = 0; i < Sample_Count; i++) {
        xyz[0] += c[i] * D65_illuminant.c[i] * CIE_X.c[i];
        xyz[1] += c[i] * D65_illuminant.c[i] * CIE_Y.c[i];
        xyz[2] += c[i] * D65_illuminant.c[i] * CIE_Z.c[i];
    }
    xyz *= (float)Interval_Length * CIE_Y_D65_integral_inverse;
    return xyz;
}

ColorRGB convert_flux_to_constant_spectrum_to_rgb_intensity(float luminous_flux) {
    float radiant_flux_per_wavelength = luminous_flux / (683.f * CIE_Y_integral); // [W/m]

    // Get constant spectrum that produces given luminous_flux.
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(radiant_flux_per_wavelength);

    Vector3 xyz_flux = s.emission_spectrum_to_XYZ();

    constexpr float uniform_radial_flux_to_intensity = 0.25f * Pi_Inv;
    Vector3 xyz_intensity = xyz_flux * uniform_radial_flux_to_intensity;

    // NOTE: Constant spectrum does not produce white RGB (for sRGB). It's a bit reddish.
    return XYZ_to_sRGB(xyz_intensity);
}
