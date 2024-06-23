#include "std.h"
#include "lib/common.h"
#include "scattering.h"

#include "lib/vector.h"

ColorRGB schlick_fresnel(const ColorRGB& R0, float cos_theta_i) {
    float k = 1.f - std::abs(cos_theta_i);
    float k5 = (k * k) * (k * k) * k;
    return R0 + (ColorRGB(1) - R0) * k5;
}

float dielectric_fresnel(float cos_theta_i, float eta) {
    cos_theta_i = std::min(std::abs(cos_theta_i), 1.f);
    float sin_theta_i = std::sqrt(1.f - cos_theta_i * cos_theta_i);
    float sin_theta_t = (1.f / eta) * sin_theta_i;

    if (sin_theta_t >= 1.f)
        return 1.f;

    float cos_theta_t = std::sqrt(1.f - sin_theta_t * sin_theta_t);

    float Rp = (eta * cos_theta_i - cos_theta_t) /
               (eta * cos_theta_i + cos_theta_t);

    float Rs = (cos_theta_i - eta * cos_theta_t) /
               (cos_theta_i + eta * cos_theta_t);

    float f = 0.5f * (Rp*Rp + Rs*Rs);
    ASSERT(f <= 1.f);
    return f;
}

ColorRGB conductor_fresnel(float cos_theta_i, float eta_i, const ColorRGB& eta_t, const ColorRGB& k_t) {
    cos_theta_i = std::abs(std::clamp(cos_theta_i, -1.f, 1.f));
    float cos_theta_i2 = cos_theta_i * cos_theta_i;
    float sin_theta_i2 = 1.f - cos_theta_i2;

    ColorRGB eta = eta_t / eta_i;
    ColorRGB k = k_t / eta_i;
    ColorRGB eta2 = eta * eta;
    ColorRGB k2 = k * k;

    auto t0 = eta2 - k2 - ColorRGB(sin_theta_i2);
    auto a2_plus_b2 = ColorRGB::sqrt(t0*t0 + 4*eta2*k2);
    auto t1 = a2_plus_b2 + ColorRGB(cos_theta_i2);
    auto a = ColorRGB::sqrt(0.5f * (a2_plus_b2 + t0));
    auto t2 = (2 * cos_theta_i) * a;
    auto Rs = (t1 - t2) / (t1 + t2);

    auto t3 = cos_theta_i2 * a2_plus_b2 + ColorRGB(sin_theta_i2 * sin_theta_i2);
    auto t4 = t2 * sin_theta_i2;
    auto Rp = Rs * (t3 - t4) / (t3 + t4);

    ColorRGB F = 0.5f * (Rp + Rs);
    return F;
}

ColorRGB microfacet_reflection(const ColorRGB& F, float G, float D, float wo_dot_n, float wi_dot_n)
{
    ColorRGB f = F * ((G * D) / (4.f * wo_dot_n * wi_dot_n));
    return f;
}

float microfacet_reflection(float F, float G, float D, float wo_dot_n, float wi_dot_n)
{
    float f = (F * G * D) / (4.f * wo_dot_n * wi_dot_n);
    return f;
}

float microfacet_transmission(float F, float G, float D,
    float wo_dot_n, float wi_dot_n, float wo_dot_wh, float wi_dot_wh,
    float eta_o, float eta_i)
{
    ASSERT(wo_dot_wh * wi_dot_wh <= 0.f);
    float k = std::abs((wo_dot_wh * wi_dot_wh) / (wo_dot_n * wi_dot_n));
    float k2 = eta_o * wo_dot_wh + eta_i * wi_dot_wh;
    float f = (eta_o * eta_o * k * G * D * (1.f - F)) / (k2 * k2);
    return f;
}

float GGX_Distribution::D(const Vector3& wh, const Vector3& n, float alpha) {
    float cos_theta = dot(wh, n);
    if (cos_theta <= 0.f)
        return 0.f;

    // The formula as specified in "Microfacet Models for Refraction through Rough Surfaces".
    // https://www.cs.cornell.edu/~srm/publications/EGSR07-btdf.pdf
    /*
    float cos2_theta = cos_theta * cos_theta;
    float a2 = alpha * alpha;
    float cos4_theta = cos2_theta * cos2_theta;
    float tan2_theta = (1.f - cos2_theta) / cos2_theta;
    float k = a2 + tan2_theta;
    float D = a2 / (Pi * cos4_theta * k * k);
    */

    // Algebraic transformation of the above code which saves 1 div and 1 mul.
    float cos2_theta = cos_theta * cos_theta;
    float a2 = alpha * alpha;
    float k = cos2_theta * (a2 - 1) + 1;
    float D = a2 / (Pi * k *k);
    return D;
}

inline float GGX_lambda(const Vector3& v, const Vector3& n, float alpha) {
    float cos_theta = dot(v, n);
    float cos2_theta = cos_theta * cos_theta;
    float tan2_theta = std::max(0.f, (1.f - cos2_theta) / cos2_theta); // could be Infinity, that's fine

    float lambda = 0.5f * (-1.f + std::sqrt(1.f + alpha * alpha * tan2_theta));
    return lambda;
}

float GGX_Distribution::G(const Vector3& wi, const Vector3& wo, const Vector3& n, float alpha) {
    return 1.f / (1.f + GGX_lambda(wi, n, alpha) + GGX_lambda(wo, n, alpha));
}

float GGX_Distribution::G1(const Vector3& v, const Vector3& n, float alpha) {
    return 1.f / (1.f + GGX_lambda(v, n, alpha));
}
