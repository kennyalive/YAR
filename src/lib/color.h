#pragma once

struct Vector3;

struct ColorRGB {
    float r;
    float g;
    float b;

    ColorRGB() : r(0), g(0), b(0) {}
    constexpr ColorRGB(float r, float g, float b): r(r), g(g), b(b) {}
    explicit ColorRGB(const float* v) : r(v[0]), g(v[1]), b(v[2]) {}

    float operator[](int index) const {
        ASSERT(index >= 0 && index < 3);
        return (&r)[index];
    }

    float& operator[](int index) {
        ASSERT(index >= 0 && index < 3);
        return (&r)[index];
    }

    void operator*=(float v) {
        r *= v;
        g *= v;
        b *= v;
    }
    void operator/=(float v) {
        ASSERT(v != 0.f);
        float inv_v = 1.f / v;
        r *= inv_v;
        g *= inv_v;
        b *= inv_v;
    }
    void operator+=(const ColorRGB c) {
        r += c.r;
        g += c.g;
        b += c.b;
    }
};

inline ColorRGB operator*(const ColorRGB& c, float k) {
    return ColorRGB{c[0] * k, c[1] * k, c[2] * k};
}

inline ColorRGB operator/(const ColorRGB& c, float k) {
    float inv_k = 1.f / k;
    return c * inv_k;
}

inline ColorRGB operator*(float v, const ColorRGB& color) {
    return color * v;
}

inline ColorRGB operator*(const ColorRGB& a, const ColorRGB& b) {
    return ColorRGB{ a[0] * b[0], a[1] * b[1], a[2] * b[2] };
}

inline ColorRGB operator+(const ColorRGB& a, const ColorRGB& b) {
    return ColorRGB{ a[0] + b[0], a[1] + b[1], a[2] + b[2] };
}

inline ColorRGB operator-(const ColorRGB& a, const ColorRGB& b) {
    return ColorRGB{ a[0] - b[0], a[1] - b[1], a[2] - b[2] };
}

// Conversion from XYZ to sRGB color space (without gamma encoding).
ColorRGB ColorRGBFromXYZ(const Vector3& xyz);

constexpr ColorRGB Color_Black = ColorRGB{0, 0, 0};
constexpr ColorRGB Color_White = ColorRGB{1, 1, 1};