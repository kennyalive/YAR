#pragma once

#include <cassert>

// disable windows RGB macro
#ifdef RGB
#undef RGB
#endif

struct XYZ {
    float c[3];

    explicit XYZ() {
        c[0] = c[1] = c[2] = 0.f;
    }
    explicit XYZ(float xyz[3]) {
        c[0] = xyz[0];
        c[1] = xyz[1];
        c[2] = xyz[2];
    }
    XYZ(float c0, float c1, float c2) {
        c[0] = c0;
        c[1] = c1;
        c[2] = c2;
    }
    float operator[](int index) const {
        assert(index >= 0 && index < 3);
        return c[index];
    }
    float& operator[](int index) {
        assert(index >= 0 && index < 3);
        return c[index];
    }
    void operator*=(float v) {
        c[0] *= v;
        c[1] *= v;
        c[2] *= v;
    }
};

struct RGB {
    float c[3];

    explicit RGB(float v = 0.f) {
        c[0] = c[1] = c[2] = v;
    }
    explicit RGB(float rgb[3]) {
        c[0] = rgb[0];
        c[1] = rgb[1];
        c[2] = rgb[2];
    }
    RGB(float c0, float c1, float c2) {
        c[0] = c0;
        c[1] = c1;
        c[2] = c2;
    }

    // Conversion from XYZ to sRGB color space (without gamma encoding).
    RGB(const XYZ& xyz);

    float operator[](int index) const {
        assert(index >= 0 && index < 3);
        return c[index];
    }
    float& operator[](int index) {
        assert(index >= 0 && index < 3);
        return c[index];
    }
    void operator*=(float v) {
        c[0] *= v;
        c[1] *= v;
        c[2] *= v;
    }
};

inline RGB operator*(const RGB& rgb, float v) {
    return RGB(rgb[0] * v, rgb[1] * v, rgb[2] * v);
}

inline RGB operator*(float v, const RGB& rgb) {
    return rgb * v;
}

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
    static Sampled_Spectrum constant_spectrum(float c);

    XYZ emission_spectrum_to_XYZ() const;
    XYZ reflectance_spectrum_to_XYZ() const;
};
