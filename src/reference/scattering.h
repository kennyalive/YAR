#pragma once

#include "lib/color.h"

struct Vector3;

// theta_i is the angle between incident direction and normal.
// For microfacet model the normal is a half-vector (wi + wo).
ColorRGB schlick_fresnel(const ColorRGB& R0, float cos_theta_i);

// eta - relative IOR (transmitted media IOR over incident media IOR)
float dielectric_fresnel(float cos_theta_i, float eta);

// theta_i is the angle between incident direction and normal.
// For microfacet model the normal is a half-vector (wi + wo).
ColorRGB conductor_fresnel(float cos_theta_i, float eta_i, const ColorRGB& eta_t, const ColorRGB& k_t);

struct GGX_Distribution {
    // wh -normalized half-vector (wi+wo)
    // n - shading normal
    static float D(const Vector3& wh, const Vector3& n, float alpha);

    // wi/wo - incident/outgoing directions
    // n - shading normal
    static float G(const Vector3& wi, const Vector3& wo, const Vector3& n, float alpha);

    // v - either wo or wi
    // n - shading normal
    static float G1(const Vector3& v, const Vector3& n, float alpha);
};
