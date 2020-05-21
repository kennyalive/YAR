#pragma once

#include "lib/color.h"
#include "lib/material.h"

struct Shading_Context;
struct Render_Context;
struct Vector3;

// theta_i is the angle between incident direction and normal.
// For microfacet model the normal is a half-vector (wi + wo).
ColorRGB schlick_fresnel(const ColorRGB& R0, float cos_theta_i);

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
};
