#include "std.h"
#include "lib/common.h"
#include "bsdf.h"
#include "bsdf_pbrt.h"

#include "parameter_evaluation.h"
#include "sampling.h"
#include "scattering.h"
#include "thread_context.h"

//
// Uber material from PBRT 3
//
Pbrt3_Uber_BRDF::Pbrt3_Uber_BRDF(const Thread_Context& thread_ctx, const Pbrt3_Uber_Material& params)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;
    opacity = evaluate_rgb_parameter(thread_ctx, params.opacity);
    diffuse_reflectance = evaluate_rgb_parameter(thread_ctx, params.diffuse_reflectance);
    specular_reflectance = evaluate_rgb_parameter(thread_ctx, params.specular_reflectance);
    alpha = ggx_alpha(thread_ctx, params.roughness, params.roughness_is_alpha);
    index_of_refraction = evaluate_float_parameter(thread_ctx, params.index_of_refraction);
}

ColorRGB Pbrt3_Uber_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    ColorRGB diffuse_brdf = Pi_Inv * diffuse_reflectance * opacity;

    Vector3 wh = (wo + wi).normalized();
    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);
    float F = dielectric_fresnel(cos_theta_i, index_of_refraction);
    float D = GGX_Distribution::D(wh, normal, alpha);
    float G = GGX_Distribution::G(wi, wo, normal, alpha);
    float f = (G * D * F) / (4.f * dot(normal, wo) * dot(normal, wi));
    ColorRGB specular_brdf = f * specular_reflectance * opacity;

    return diffuse_brdf + specular_brdf;
}

ColorRGB Pbrt3_Uber_BRDF::sample(Vector2 u, float u_scattering_type, const Vector3& wo, Vector3* wi, float* pdf) const
{
    if (u_scattering_type < 0.5f) { // sample diffuse
        Vector3 local_dir = sample_hemisphere_cosine(u);
        *wi = local_to_world(local_dir);
    }
    else { // sample specular
        Vector3 wh = sample_microfacet_normal(u, wo, alpha);
        *wi = reflect(wo, wh);
    }

    if (dot(normal, *wi) <= 0.f) {
        return Color_Black;
    }
    *pdf = Pbrt3_Uber_BRDF::pdf(wo, *wi);
    return Pbrt3_Uber_BRDF::evaluate(wo, *wi);
}

float Pbrt3_Uber_BRDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    ASSERT(dot(normal, wi) >= 0.f);
    float diffuse_pdf = dot(normal, wi) / Pi;

    Vector3 wh = (wo + wi).normalized();
    float specular_pdf = calculate_microfacet_reflection_wi_pdf(wo, wh, normal, alpha);

    float pdf = 0.5f * (diffuse_pdf + specular_pdf);
    return pdf;
}

//
// Pbrt3 Plastic BRDF
//
Pbrt3_Plastic_BRDF::Pbrt3_Plastic_BRDF(const Thread_Context& thread_ctx, const Plastic_Material& params)
    : Plastic_BRDF(thread_ctx, params)
{
    original_shading_normal = thread_ctx.shading_context.original_shading_normal_was_flipped ? -normal : normal;
}

ColorRGB Pbrt3_Plastic_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    // In pbrt3 eta_t is 1.0 and eta_i is 1.5, which is a bug but it became a feature.
    // We need to do the same to produce pbrt3 output.
    float relative_ior = 1.f / 1.5f;

    bool flip_ior = dot(original_shading_normal, wi) < 0.f;
    if (flip_ior)
        relative_ior = 1.5f / 1.f;

    ColorRGB F = ColorRGB(dielectric_fresnel(cos_theta_i, relative_ior));

    float D = GGX_Distribution::D(wh, normal, alpha);
    float G = GGX_Distribution::G(wi, wo, normal, alpha);

    ColorRGB specular_brdf = (G * D) * F * r0 / (4.f * dot(normal, wo) * dot(normal, wi));
    ColorRGB diffuse_brdf = diffuse_reflectance * Pi_Inv;
    return diffuse_brdf + specular_brdf;
}

//
// Pbrt3 Fourier BRDF
//
Pbrt3_Fourier_BSDF::Pbrt3_Fourier_BSDF(const Thread_Context& thread_ctx, const Pbrt3_Fourier_Material& params)
    : BSDF(thread_ctx.shading_context)
    , data(params)
{
    reflection_scattering = true;
    ASSERT(data.eta == 1.f); // support only reflection
}

ColorRGB Pbrt3_Fourier_BSDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    // fourier bsdf data uses inversed incident direction comparing to our representation
    // (both incident and outgoing directions point away from the surface)
    float cos_i = std::clamp(dot(normal, -wi), -1.f, 1.f);

    float cos_o = std::clamp(dot(normal, wo), -1.f, 1.f);

    auto find_index = [this](float cos_theta) -> int {
        const auto& cosines = data.zenith_angle_discretization;
        auto it = std::lower_bound(cosines.begin(), cosines.end(), cos_theta);
        ASSERT(it != cosines.end());
        return int(it - cosines.begin());
        };
    int index_i = find_index(cos_i);
    int index_o = find_index(cos_o);

    int index = index_o * (int)data.zenith_angle_discretization.size() + index_i;

    uint32_t coeff_count = data.coeff_count[index];
    const float* coeffs = data.coeffs.data() + data.coeff_offset[index];

    float cos_phi = cos_delta_phi(wo, -wi, tangent, bitangent);
    float phi = std::acos(cos_phi);

    // the fourier series computes bsdf * abs(cos_i), so we need to remove cosine
    float scale = (cos_i != 0.f) ? 1.f / std::abs(cos_i) : 0.f;

    ASSERT(data.channel_count == 1 || data.channel_count == 3);

    if (data.channel_count == 1) {
        float y = 0.f;
        for (uint32_t i = 0; i < coeff_count; i++) {
            float coeff_cos = std::cos(float(i) * phi);
            y += coeffs[i] * coeff_cos;
        }
        ColorRGB f(y * scale);
        return f;
    }
    else {
        float y = 0.f;
        float r = 0.f;
        float b = 0.f;
        for (uint32_t i = 0; i < coeff_count; i++) {
            float coeff_cos = std::cos(float(i) * phi);
            y += coeffs[i + 0 * coeff_count] * coeff_cos;
            r += coeffs[i + 1 * coeff_count] * coeff_cos;
            b += coeffs[i + 2 * coeff_count] * coeff_cos;
        }
        float g = get_green_from_YRB(y, r, b);
        ColorRGB f(r * scale, g * scale, b * scale);
        return f;
    }
}

ColorRGB Pbrt3_Fourier_BSDF::sample(Vector2 u, float u_scattering_type, const Vector3& wo, Vector3* wi, float* pdf) const
{
    Vector3 local_dir = sample_hemisphere_cosine(u);
    *wi = local_to_world(local_dir);
    *pdf = Pbrt3_Fourier_BSDF::pdf(wo, *wi);
    return Pbrt3_Fourier_BSDF::evaluate(wo, *wi);
}

float Pbrt3_Fourier_BSDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    ASSERT(dot(normal, wi) >= 0.f);
    return dot(normal, wi) / Pi; // pdf for cosine-weighted hemisphere sampling
}
