#include "std.h"
#include "lib/common.h"
#include "scattering.h"

#include "scene_context.h"
#include "thread_context.h"

#include "lib/vector.h"

ColorRGB schlick_fresnel(const ColorRGB& R0, float cos_theta_i)
{
    float k = 1.f - std::abs(cos_theta_i);
    float k5 = (k * k) * (k * k) * k;
    return R0 + (ColorRGB(1) - R0) * k5;
}

float dielectric_fresnel(float cos_theta_i, float eta)
{
    cos_theta_i = std::min(std::abs(cos_theta_i), 1.f);
    float sin_theta_i = std::sqrt(1.f - cos_theta_i * cos_theta_i);
    float sin_theta_t = (1.f / eta) * sin_theta_i;
    if (sin_theta_t >= 1.f) {
        return 1.f;
    }

    float cos_theta_t = std::sqrt(1.f - sin_theta_t * sin_theta_t);

    float Rp = (eta * cos_theta_i - cos_theta_t) /
               (eta * cos_theta_i + cos_theta_t);

    float Rs = (cos_theta_i - eta * cos_theta_t) /
               (cos_theta_i + eta * cos_theta_t);

    float f = 0.5f * (Rp*Rp + Rs*Rs);
    ASSERT(f <= 1.f);
    return f;
}

ColorRGB conductor_fresnel(float cos_theta_i, float eta_i, const ColorRGB& eta_t, const ColorRGB& k_t)
{
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

Vector3 refraction_half_direction(float eta_o, const Vector3& wo, float eta_i, const Vector3& wi, const Vector3& normal)
{
    // The following formula computes half-direction for refraction.
    // The computed vector points into the hemisphere with a smaller index of refraction.
    Vector3 wh = -(eta_o * wo + eta_i * wi).normalized();

    // Enforce convention the resulted vector is in the hemisphere defined by the normal.
    if (dot(wh, normal) < 0.f) {
        wh = -wh;
    }
    return wh;
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

// The probability density calculations for reflection and tranmission are from the classic paper:
// "Microfacet Models for Refraction through Rough Surfaces"
// https://www.cs.cornell.edu/~srm/publications/EGSR07-btdf.pdf
// I rederived these formulas to check there are no typos, and also adjusted variables
// naming according to the conventions of this renderer.

float microfacet_reflection_wi_pdf(const Vector3& wo, const Vector3& wh, const Vector3& n, float alpha)
{
    float wh_pdf = GGX_visible_microfacet_normal_pdf(wo, wh, n, alpha);

    // Convert between probability densities:
    //  wi_pdf = wh_pdf * dwh/dwi
    //  dwh/dwi = 1/4(wh, wi) = 1/4(wh,wo)
    float wi_pdf = wh_pdf / (4 * dot(wh, wo));
    return wi_pdf;
}

float microfacet_reflection_wi_pdf_anisotropic(const Vector3& wo_local, const Vector3& wh_local, float alpha_x, float alpha_y)
{
    float wh_pdf = GGX_visible_microfacet_normal_pdf_anisotropic(wo_local, wh_local, alpha_x, alpha_y);

    // Convert between probability densities:
    //  wi_pdf = wh_pdf * dwh/dwi
    //  dwh/dwi = 1/4(wh, wi) = 1/4(wh,wo)
    float wi_pdf = wh_pdf / (4 * dot(wh_local, wo_local));
    return wi_pdf;
}

float microfacet_transmission_wi_pdf(const Vector3& wo, const Vector3& wi, const Vector3& wh, const Vector3& n, float alpha, float eta_o, float eta_i)
{
    // The computation of transmission half-angle direction gets a vector that is in
    // the hemisphere with a lower index of refraction. If the computed vector is not
    // in the hemisphere defined by the normal, the caller should flip it before
    // calling this function.
    ASSERT(dot(wh, n) >= 0.f);

    // The wo/wi vectors should be on the opposite side of the half-angle direction to be
    // able to form a refraction configuration.
    ASSERT(dot(wo, wh) * dot(wi, wh) <= 0.f);

    float wh_pdf = GGX_visible_microfacet_normal_pdf(wo, wh, n, alpha);

    // Convert between probability densities:
    //  wi_pdf = wh_pdf * dwh/dwi
    //  dwh/dwi = eta_i^2 * abs(dot(wi, wh)) / (eta_o*dot(wo, wh) + eta_i*dot(wi, wh))^2
    float denom = eta_o * dot(wo, wh) + eta_i * dot(wi, wh);
    float dwh_over_dwi = eta_i * eta_i * std::abs(dot(wi, wh)) / (denom * denom);

    float wi_pdf = wh_pdf * dwh_over_dwi;
    return wi_pdf;
}

float GGX_Distribution::D(const Vector3& wh, const Vector3& n, float alpha)
{
    float cos_theta = dot(wh, n);
    if (cos_theta <= 0.f) {
        return 0.f;
    }

    // The formula as specified in "Microfacet Models for Refraction through Rough Surfaces".
    // Section 5.2, GGX Distribution.
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

float GGX_Distribution::D_anisotropic(const Vector3& wh_local, float alpha_x, float alpha_y)
{
    float cos_theta = std::clamp(wh_local.z, -1.f, 1.f);
    float cos2_theta = cos_theta * cos_theta;
    float sin2_theta = 1.f - cos2_theta;

    float sin2_theta_inv = 1.f / sin2_theta;
    float cos2_phi = (sin2_theta == 0.f) ? 1.f : (wh_local.x * wh_local.x) * sin2_theta_inv;
    float sin2_phi = (sin2_theta == 0.f) ? 0.f : (wh_local.y * wh_local.y) * sin2_theta_inv;

    float k = cos2_theta + sin2_theta * (cos2_phi / (alpha_x * alpha_x) + sin2_phi / (alpha_y * alpha_y));
    return 1.f / (Pi * alpha_x * alpha_y * k * k);
}

static float GGX_lambda(const Vector3& v, const Vector3& n, float alpha)
{
    float cos_theta = dot(v, n);
    float cos2_theta = cos_theta * cos_theta;
    float tan2_theta = std::max(0.f, (1.f - cos2_theta) / cos2_theta); // could be Infinity, that's fine

    float lambda = 0.5f * (-1.f + std::sqrt(1.f + alpha * alpha * tan2_theta));
    return lambda;
}

static float GGX_lambda_anisotropic(const Vector3& v_local, float alpha_x, float alpha_y)
{
    float cos_theta = std::clamp(v_local.z, -1.f, 1.f);
    float cos2_theta = cos_theta * cos_theta;
    float sin2_theta = 1.f - cos2_theta;
    float tan2_theta = sin2_theta / cos2_theta; // could be Infinity, that's fine

    float sin2_theta_inv = 1.f / sin2_theta;
    float cos2_phi = (sin2_theta == 0.f) ? 1.f : (v_local.x * v_local.x) * sin2_theta_inv;
    float sin2_phi = (sin2_theta == 0.f) ? 0.f : (v_local.y * v_local.y) * sin2_theta_inv;
    float alpha2 = cos2_phi * alpha_x * alpha_x + sin2_phi * alpha_y * alpha_y;

    float lambda = 0.5f * (-1.f + std::sqrt(1.f + alpha2 * tan2_theta));
    return lambda;
}

float GGX_Distribution::G(const Vector3& wi, const Vector3& wo, const Vector3& n, float alpha)
{
    return 1.f / (1.f + GGX_lambda(wi, n, alpha) + GGX_lambda(wo, n, alpha));
}

float GGX_Distribution::G_anisotropic(const Vector3& wi_local, const Vector3& wo_local, float alpha_x, float alpha_y)
{
    return 1.f / (1.f + GGX_lambda_anisotropic(wi_local, alpha_x, alpha_y) + GGX_lambda_anisotropic(wo_local, alpha_x, alpha_y));
}

float GGX_Distribution::G1(const Vector3& v, const Vector3& n, float alpha)
{
    return 1.f / (1.f + GGX_lambda(v, n, alpha));
}

float GGX_Distribution::G1_anisotropic(const Vector3& v_local, float alpha_x, float alpha_y)
{
    return 1.f / (1.f + GGX_lambda_anisotropic(v_local, alpha_x, alpha_y));
}

static float pbrt3_roughness_to_alpha(float roughness)
{
    roughness = std::max(roughness, 1e-3f);
    float x = std::log(roughness);
    float alpha =
        1.621420000f +
        0.819955000f * x +
        0.173400000f * x * x +
        0.017120100f * x * x * x +
        0.000640711f * x * x * x * x;
    return alpha;
}

float GGX_Distribution::roughness_to_alpha(const Thread_Context& thread_ctx, float roughness, bool no_remapping)
{
    float alpha;
    if (no_remapping) {
        alpha = roughness;
    }
    else if (thread_ctx.scene_context.pbrt3_scene) {
        alpha = pbrt3_roughness_to_alpha(roughness);
    }
    else if (thread_ctx.scene_context.pbrt4_scene) {
        alpha = std::sqrt(roughness);
    }
    else {
        alpha = roughness * roughness;
    }
    return alpha;
}
