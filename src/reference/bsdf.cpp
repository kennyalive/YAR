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

// Converts between probability densities:
// wi_pdf = wh_pdf * dWh/dWi
// dWh/dWi = 1/4(wh, wi) = 1/4(wh,wo)
inline float wh_pdf_to_wi_pdf(float wh_pdf, const Vector3& wh, const Vector3& wo) {
    return wh_pdf / (4 * dot(wh, wo));
}

const BSDF* create_bsdf(const Scene_Context& scene_ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, Material_Handle material) {
    switch (material.type) {
        case Material_Type::lambertian:
        {
            const Lambertian_Material& params = scene_ctx.materials.lambertian[material.index];
            void* bsdf_allocation = thread_ctx.memory_pool.allocate<Lambertian_BRDF>();
            return new (bsdf_allocation) Lambertian_BRDF(scene_ctx, shading_ctx, params);
        }
        case Material_Type::metal:
        {
            const Metal_Material& params = scene_ctx.materials.metal[material.index];
            void* bsdf_allocation = thread_ctx.memory_pool.allocate<Metal_BRDF>();
            return new (bsdf_allocation) Metal_BRDF(scene_ctx, shading_ctx, params);
        }
        case Material_Type::plastic:
        {
            const Plastic_Material& params = scene_ctx.materials.plastic[material.index];
            void* bsdf_allocation = thread_ctx.memory_pool.allocate<Plastic_BRDF>();
            return new (bsdf_allocation) Plastic_BRDF(scene_ctx, shading_ctx, params);
        }
        default:
        {
            ASSERT(false);
            return nullptr;
        }
    }
}

BSDF::BSDF(const Shading_Context& shading_ctx)
    : shading_ctx(&shading_ctx)
    , N(shading_ctx.N)
{
}

//
// Lambertian BRDF
//
Lambertian_BRDF::Lambertian_BRDF(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const Lambertian_Material& material)
    : BSDF(shading_ctx)
{
    reflection_scattering = true;
    reflectance = evaluate_rgb_parameter(scene_ctx, shading_ctx, material.reflectance);
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
    ASSERT(dot(N, wi) >= 0.f);
    return dot(N, wi) / Pi; // pdf for cosine-weighted hemisphere sampling
}

//
// Metal BRDF
//
Metal_BRDF::Metal_BRDF(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const Metal_Material& material)
    : BSDF(shading_ctx)
{
    this->scene_ctx = &scene_ctx;
    reflection_scattering = true;

    float roughness = evaluate_float_parameter(scene_ctx, shading_ctx, material.roughness);
    alpha = roughness * roughness;

    eta_i = evaluate_float_parameter(scene_ctx, shading_ctx, material.eta_i);
    eta_t = evaluate_rgb_parameter(scene_ctx, shading_ctx, material.eta);
    k_t = evaluate_rgb_parameter(scene_ctx, shading_ctx, material.k);
}

ColorRGB Metal_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const {
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = conductor_fresnel(cos_theta_i, eta_i, eta_t, k_t);

    float D = GGX_Distribution::D(wh, shading_ctx->N, alpha);
    float G = GGX_Distribution::G(wi, wo, shading_ctx->N, alpha);
    
    ColorRGB f = (G * D) * F / (4.f * dot(shading_ctx->N, wo) * dot(shading_ctx->N, wi));
    return f;
}

ColorRGB Metal_BRDF::sample(Vector2 u, const Vector3& wo, Vector3* wi, float* pdf) const {
    ASSERT_ZERO_TO_ONE_RANGE_VECTOR2(u);

    Vector3 wh_local;
    if (ggx_sample_visible_normals) {
        Vector3 wo_local = shading_ctx->world_to_local(wo);
        wh_local = GGX_sample_visible_microfacet_normal(u, wo_local, alpha, alpha);
    }
    else {
        wh_local = GGX_sample_microfacet_normal(u, alpha);
    }

    Vector3 wh = shading_ctx->local_to_world(wh_local);
    ASSERT(dot(wh, N) >= 0.f);

    *wi = wh * (2.f * dot(wo, wh)) - wo;

    if (dot(N, *wi) <= 0.f)
        return Color_Black;

    float wh_pdf;
    if (ggx_sample_visible_normals)
        wh_pdf = GGX_visible_microfacet_normal_pdf(wo, wh, N, alpha);
    else
        wh_pdf = GGX_microfacet_normal_pdf(wh, N, alpha);

    *pdf = wh_pdf_to_wi_pdf(wh_pdf, wh, wo);

    return evaluate(wo, *wi);
}

float Metal_BRDF::pdf(const Vector3& wo, const Vector3& wi) const {
    ASSERT(dot(N, wi) >= 0.f);
    Vector3 wh = (wo + wi).normalized();

    float wh_pdf;
    if (ggx_sample_visible_normals)
        wh_pdf = GGX_visible_microfacet_normal_pdf(wo, wh, N, alpha);
    else
        wh_pdf = GGX_microfacet_normal_pdf(wh, N, alpha);

    float wi_pdf = wh_pdf_to_wi_pdf(wh_pdf, wh, wo);
    return wi_pdf;
}

//
// Plastic BRDF
//
Plastic_BRDF::Plastic_BRDF(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const Plastic_Material& params)
    : BSDF(shading_ctx)
{
    this->scene_ctx = &scene_ctx;

    reflection_scattering = true;

    roughness = evaluate_float_parameter(scene_ctx, shading_ctx, params.roughness);
    r0 = evaluate_float_parameter(scene_ctx, shading_ctx, params.r0);
    diffuse_reflectance = evaluate_rgb_parameter(scene_ctx, shading_ctx, params.diffuse_reflectance);
}

ColorRGB Plastic_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const {
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = schlick_fresnel(ColorRGB(0.04f), cos_theta_i);

    float alpha = roughness * roughness;
    float D = GGX_Distribution::D(wh, shading_ctx->N, alpha);
    float G = GGX_Distribution::G(wi, wo, shading_ctx->N, alpha);

    ColorRGB specular_brdf = (G * D) * F * r0 / (4.f * dot(shading_ctx->N, wo) * dot(shading_ctx->N, wi));
    ColorRGB diffuse_brdf = diffuse_reflectance * Pi_Inv;
    return diffuse_brdf + specular_brdf;
}
