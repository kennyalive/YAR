#pragma once

#include "lib/color.h"

struct Thread_Context;
struct Vector3;

// theta_i is the angle between incident direction and normal.
// For microfacet model the normal is a half-vector (wi + wo).
ColorRGB schlick_fresnel(const ColorRGB& R0, float cos_theta_i);

// eta - relative IOR (transmitted media IOR over incident media IOR)
float dielectric_fresnel(float cos_theta_i, float eta);

// theta_i is the angle between incident direction and normal.
// For microfacet model the normal is a half-vector (wi + wo).
ColorRGB conductor_fresnel(float cos_theta_i, float eta_i, const ColorRGB& eta_t, const ColorRGB& k_t);

Vector3 refraction_half_direction(float eta_o, const Vector3& wo, float eta_i, const Vector3& wi, const Vector3& normal);

ColorRGB microfacet_reflection(const ColorRGB& F, float G, float D, float wo_dot_n, float wi_dot_n);
float microfacet_reflection(float F, float G, float D, float wo_dot_n, float wi_dot_n);

float microfacet_transmission(float F, float G, float D,
    float wo_dot_n, float wi_dot_n, float wo_dot_wh, float wi_dot_wh,
    float eta_o, float eta_i);

float microfacet_reflection_wi_pdf(const Vector3& wo, const Vector3& wh, const Vector3& n, float alpha);
float microfacet_transmission_wi_pdf(const Vector3& wo, const Vector3& wi, const Vector3& wh, const Vector3& n, float alpha, float eta_o, float eta_i);

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

    // 'roughness' is a user-friendly value from [0..1] range and the remapping
    // function converts it to 'alpha' parameter from the ggx microfacet distribution.
    // The expectation is that 'roughness' behaves perceptually more linearly than
    // distribution's alpha parameter.
    static float roughness_to_alpha(const Thread_Context& thread_ctx, float roughness, bool no_remapping);
};
