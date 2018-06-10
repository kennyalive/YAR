#pragma once

struct RGB_Spectrum {
    float c[3];

    explicit RGB_Spectrum(float v) {
        c[0] = c[1] = c[2] = v;
    }

    RGB_Spectrum(float c0, float c1, float c2) {
        c[0] = c0;
        c[1] = c1;
        c[2] = c2;
    }
};

// Sampled_Spectrum approximates spectrum function as a sequence of samples where each sample
// represents an average of the spectrum function over the interval of a fixed length.
struct Sampled_Spectrum {
    static constexpr int Wavelength_Range_Start = 380;
    static constexpr int Wavelength_Range_End = 730;

    static constexpr int Interval_Length = 5;
    static constexpr int Sample_Count = (Wavelength_Range_End - Wavelength_Range_Start) / Interval_Length;

    static_assert((Wavelength_Range_End - Wavelength_Range_Start) % Interval_Length == 0, "There should be an integral number of samples in the sampled range");

    float c[Sample_Count];

    static Sampled_Spectrum from_tabulated_data(const float* lambdas, const float* values, int n);
};
