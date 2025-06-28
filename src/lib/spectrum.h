#pragma once

#include "color.h"
#include "vector.h"

// Sampled_Spectrum approximates spectrum function as a sequence of samples where each sample
// represents an average of the spectrum function over the interval of a fixed length.
struct Sampled_Spectrum {
    static constexpr int Wavelength_Range_Start = 380;
    static constexpr int Wavelength_Range_End = 730;
    static constexpr int Interval_Length = 5;
    static constexpr int Sample_Count = (Wavelength_Range_End - Wavelength_Range_Start) / Interval_Length;

    static_assert((Wavelength_Range_End - Wavelength_Range_Start) % Interval_Length == 0, "There should be an integral number of samples in the sampled range");

    // Spectrum per-interval values
    float c[Sample_Count];

    static Sampled_Spectrum from_tabulated_data(const float* lambdas, const float* values, int n);
    static Sampled_Spectrum constant_spectrum(float c);

    // The temperature is in Kelvin (e.g. 2700K).
    static Sampled_Spectrum blackbody_normalized_spectrum(float temperature);

    void apply_scale(float scale);

    Vector3 emission_spectrum_to_XYZ() const;
    Vector3 emission_spectrum_to_XYZ_scale_by_CIE_Y_integral() const;
    Vector3 reflectance_spectrum_to_XYZ_for_D65_illuminant() const;
};

ColorRGB convert_flux_to_constant_spectrum_to_rgb_intensity(float luminous_flux); 
