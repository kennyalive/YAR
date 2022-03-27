#include "std.h"
#include "lib/common.h"
#include "bsdf.h"

#include "context.h"
#include "parameter_evaluation.h"
#include "sampling.h"
#include "scattering.h"
#include "shading_context.h"

#include "lib/math.h"

static bool ggx_sample_visible_normals = false;

inline Vector3 sample_microfacet_normal(const Shading_Context& shading_ctx, Vector2 u, const Vector3& wo, float alpha) {
    Vector3 wh_local;
    if (ggx_sample_visible_normals) {
        Vector3 wo_local = shading_ctx.world_to_local(wo);
        wh_local = GGX_sample_visible_microfacet_normal(u, wo_local, alpha, alpha);
    }
    else {
        wh_local = GGX_sample_microfacet_normal(u, alpha);
    }
    Vector3 wh = shading_ctx.local_to_world(wh_local);
    ASSERT(dot(wh, shading_ctx.normal) >= 0.f);
    return wh;
}

inline float calculate_microfacet_wi_pdf(const Vector3& wo, const Vector3& wh, const Vector3& n, float alpha) {
    float wh_pdf;
    if (ggx_sample_visible_normals)
        wh_pdf = GGX_visible_microfacet_normal_pdf(wo, wh, n, alpha);
    else
        wh_pdf = GGX_microfacet_normal_pdf(wh, n, alpha);

    // Converts between probability densities:
    // wi_pdf = wh_pdf * dWh/dWi
    // dWh/dWi = 1/4(wh, wi) = 1/4(wh,wo)
    float wi_pdf = wh_pdf / (4 * dot(wh, wo));
    return wi_pdf;
}

BSDF::BSDF(const Shading_Context& shading_ctx)
    : shading_ctx(&shading_ctx)
    , n(shading_ctx.normal)
{
}

//
// Lambertian BRDF
//
Lambertian_BRDF::Lambertian_BRDF(const Thread_Context& thread_ctx, const Lambertian_Material& material)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;
    reflectance = evaluate_rgb_parameter(thread_ctx, material.reflectance);
}

ColorRGB Lambertian_BRDF::evaluate(const Vector3& /*wo*/, const Vector3& /*wi*/) const {
    return Pi_Inv * reflectance;
}

ColorRGB Lambertian_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const {
    Vector3 local_dir = sample_hemisphere_cosine(u);
    *wi = shading_ctx->local_to_world(local_dir);
    *pdf = Lambertian_BRDF::pdf(wo, *wi);
    return Pi_Inv * reflectance;
}

float Lambertian_BRDF::pdf(const Vector3& /*wo*/, const Vector3& wi) const {
    ASSERT(dot(n, wi) >= 0.f);
    return dot(n, wi) / Pi; // pdf for cosine-weighted hemisphere sampling
}

//
// Metal BRDF
//
Metal_BRDF::Metal_BRDF(const Thread_Context& thread_ctx, const Metal_Material& material)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;

    float roughness = evaluate_float_parameter(thread_ctx, material.roughness);
    alpha = roughness * roughness;

    eta_i = evaluate_float_parameter(thread_ctx, material.eta_i);
    eta_t = evaluate_rgb_parameter(thread_ctx, material.eta);
    k_t = evaluate_rgb_parameter(thread_ctx, material.k);
}

ColorRGB Metal_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const {
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = conductor_fresnel(cos_theta_i, eta_i, eta_t, k_t);

    float D = GGX_Distribution::D(wh, n, alpha);
    float G = GGX_Distribution::G(wi, wo, n, alpha);
    
    ColorRGB f = (G * D) * F / (4.f * dot(n, wo) * dot(n, wi));
    return f;
}

ColorRGB Metal_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const {
    ASSERT_ZERO_TO_ONE_RANGE_VECTOR2(u);

    Vector3 wh = sample_microfacet_normal(*shading_ctx, u, wo, alpha);
    *wi = reflect(wo, wh);

    if (dot(n, *wi) <= 0.f)
        return Color_Black;

    *pdf = calculate_microfacet_wi_pdf(wo, wh, n, alpha);
    return evaluate(wo, *wi);
}

float Metal_BRDF::pdf(const Vector3& wo, const Vector3& wi) const {
    ASSERT(dot(n, wi) >= 0.f);
    Vector3 wh = (wo + wi).normalized();
    return calculate_microfacet_wi_pdf(wo, wh, n, alpha);
}

//
// Plastic BRDF
//
Plastic_BRDF::Plastic_BRDF(const Thread_Context& thread_ctx, const Plastic_Material& params)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;

    float roughness = evaluate_float_parameter(thread_ctx, params.roughness);
    alpha = roughness * roughness;

    r0 = evaluate_float_parameter(thread_ctx, params.r0);
    diffuse_reflectance = evaluate_rgb_parameter(thread_ctx, params.diffuse_reflectance);
}

ColorRGB Plastic_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const {
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = schlick_fresnel(ColorRGB(0.04f), cos_theta_i);

    float D = GGX_Distribution::D(wh, n, alpha);
    float G = GGX_Distribution::G(wi, wo, n, alpha);

    ColorRGB specular_brdf = (G * D) * F * r0 / (4.f * dot(n, wo) * dot(n, wi));
    ColorRGB diffuse_brdf = diffuse_reflectance * Pi_Inv;
    return diffuse_brdf + specular_brdf;
}

ColorRGB Plastic_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const {
    if (u[0] < 0.5f) { // sample diffuse
        u[0] *= 2.f; // remap u[0] to [0, 1) range

        Vector3 local_dir = sample_hemisphere_cosine(u);
        *wi = shading_ctx->local_to_world(local_dir);
    }
    else { // sample specular
        u[0] = (u[0] - 0.5f) * 2.f; // remap u[0] to [0, 1) range
        Vector3 wh = sample_microfacet_normal(*shading_ctx, u, wo, alpha);
        *wi = reflect(wo, wh);
    }

    if (dot(n, *wi) <= 0.f)
        return Color_Black;

    *pdf = Plastic_BRDF::pdf(wo, *wi);
    return evaluate(wo, *wi);
}

float Plastic_BRDF::pdf(const Vector3& wo, const Vector3& wi) const {
    ASSERT(dot(n, wi) >= 0.f);
    float diffuse_pdf = dot(n, wi) / Pi;

    Vector3 wh = (wo + wi).normalized();
    float spec_pdf = calculate_microfacet_wi_pdf(wo, wh, n, alpha);

    float pdf = 0.5f * (diffuse_pdf + spec_pdf);
    return pdf;
}

//
// Ashikhmin_Shirley_Phong_BRDF
//
// BRDF described in "An Anisotropic Phong Light Reflection Model", Michael Ashikhmin, Peter Shirley.
// https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.18.4504&rep=rep1&type=pdf
//
Ashikhmin_Shirley_Phong_BRDF::Ashikhmin_Shirley_Phong_BRDF(const Thread_Context& thread_ctx, const Coated_Diffuse_Material& params)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;

    float roughness = evaluate_float_parameter(thread_ctx, params.roughness);
    alpha = roughness * roughness;

    r0 = evaluate_rgb_parameter(thread_ctx, params.r0);
    diffuse_reflectance = evaluate_rgb_parameter(thread_ctx, params.diffuse_reflectance);
}

ColorRGB Ashikhmin_Shirley_Phong_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const {
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = schlick_fresnel(r0, cos_theta_i);
    float D = GGX_Distribution::D(wh, n, alpha);

    ColorRGB specular_brdf = F * (D / (4.f * cos_theta_i * std::max(dot(n, wo), dot(n, wi))));

    auto pow5 = [](float v) { return (v * v) * (v * v) * v; };

    ColorRGB diffuse_brdf =
            (diffuse_reflectance * (ColorRGB(1.f) - r0)) *
            (28.f / (23.f*Pi) * (1.f - pow5(1.f - 0.5f * dot(n, wi))) * (1.f - pow5(1.f - 0.5f * dot(n, wo))));

    return diffuse_brdf + specular_brdf;
}

ColorRGB Ashikhmin_Shirley_Phong_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const {
    if (u[0] < 0.5f) { // sample diffuse
        u[0] *= 2.f; // remap u[0] to [0, 1) range

        Vector3 local_dir = sample_hemisphere_cosine(u);
        *wi = shading_ctx->local_to_world(local_dir);
    }
    else { // sample specular
        u[0] = (u[0] - 0.5f) * 2.f; // remap u[0] to [0, 1) range
        Vector3 wh = sample_microfacet_normal(*shading_ctx, u, wo, alpha);
        *wi = reflect(wo, wh);
    }

    if (dot(n, *wi) <= 0.f)
        return Color_Black;

    *pdf = Ashikhmin_Shirley_Phong_BRDF::pdf(wo, *wi);
    return evaluate(wo, *wi);
}

float Ashikhmin_Shirley_Phong_BRDF::pdf(const Vector3& wo, const Vector3& wi) const {
    ASSERT(dot(n, wi) >= 0.f);
    float diffuse_pdf = dot(n, wi) / Pi;

    Vector3 wh = (wo + wi).normalized();
    float spec_pdf = calculate_microfacet_wi_pdf(wo, wh, n, alpha);

    float pdf = 0.5f * (diffuse_pdf + spec_pdf);
    return pdf;
}

//
// Uber material from PBRT 3
//
Pbrt3_Uber_BRDF::Pbrt3_Uber_BRDF(const Thread_Context& thread_ctx, const Pbrt3_Uber_Material& params)
    : BSDF(thread_ctx.shading_context)
{
    reflection_scattering = true;
    // NOTE: uber material also supports perfect specular transmission.
    // As in other parts of this renderer delta scattering is handled by a dedicated
    // code (specular_scattering.h/cpp) and BSDFs represent only finite functions.

    diffuse_reflectance = evaluate_rgb_parameter(thread_ctx, params.diffuse_reflectance);
    specular_reflectance = evaluate_rgb_parameter(thread_ctx, params.specular_reflectance);
    ASSERT(specular_reflectance == Color_Black); // TODO: implement specular scattering
    opacity = evaluate_rgb_parameter(thread_ctx, params.opacity);
}

ColorRGB Pbrt3_Uber_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    ColorRGB diffuse_brdf = Pi_Inv * diffuse_reflectance * opacity;
    return diffuse_brdf;
}

ColorRGB Pbrt3_Uber_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const
{
    Vector3 local_dir = sample_hemisphere_cosine(u);
    *wi = shading_ctx->local_to_world(local_dir);
    *pdf = Pbrt3_Uber_BRDF::pdf(wo, *wi);
    return Pbrt3_Uber_BRDF::evaluate(wo, *wi);
}

float Pbrt3_Uber_BRDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    ASSERT(dot(n, wi) >= 0.f);
    float cosine_distribution_pdf = dot(n, wi) / Pi; // pdf for cosine-weighted hemisphere sampling
    float finite_term_selection_probability = opacity.luminance();
    return cosine_distribution_pdf * finite_term_selection_probability;
}

const BSDF* create_bsdf(Thread_Context& thread_ctx, Material_Handle material) {
    const Scene_Context& scene_ctx = *thread_ctx.scene_context;
    switch (material.type) {
    case Material_Type::lambertian:
    {
        const Lambertian_Material& params = scene_ctx.materials.lambertian[material.index];
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Lambertian_BRDF>();
        return new (bsdf_allocation) Lambertian_BRDF(thread_ctx, params);
    }
    case Material_Type::metal:
    {
        const Metal_Material& params = scene_ctx.materials.metal[material.index];
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Metal_BRDF>();
        return new (bsdf_allocation) Metal_BRDF(thread_ctx, params);
    }
    case Material_Type::plastic:
    {
        const Plastic_Material& params = scene_ctx.materials.plastic[material.index];
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Plastic_BRDF>();
        return new (bsdf_allocation) Plastic_BRDF(thread_ctx, params);
    }
    case Material_Type::coated_diffuse:
    {
        const Coated_Diffuse_Material& params = scene_ctx.materials.coated_diffuse[material.index];
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Ashikhmin_Shirley_Phong_BRDF>();
        return new (bsdf_allocation) Ashikhmin_Shirley_Phong_BRDF(thread_ctx, params);
    }
    case Material_Type::pbrt3_uber:
    {
        const Pbrt3_Uber_Material& params = scene_ctx.materials.pbrt3_uber[material.index];
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Pbrt3_Uber_BRDF>();
        return new (bsdf_allocation) Pbrt3_Uber_BRDF(thread_ctx, params);
    }
    default:
    {
        ASSERT(false);
        return nullptr;
    }
    }
}
