#include "std.h"
#include "lib/common.h"
#include "bsdf.h"

#include "context.h"
#include "parameter_evaluation.h"
#include "sampling.h"
#include "scattering.h"
#include "shading_context.h"

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
    this->shading_ctx = &shading_ctx;

    roughness = evaluate_float_parameter(scene_ctx, shading_ctx, material.roughness);
    eta_i = evaluate_float_parameter(scene_ctx, shading_ctx, material.eta_i);
    eta_t = evaluate_rgb_parameter(scene_ctx, shading_ctx, material.eta);
    k_t = evaluate_rgb_parameter(scene_ctx, shading_ctx, material.k);
}

ColorRGB Metal_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const {
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = conductor_fresnel(cos_theta_i, eta_i, eta_t, k_t);

    float alpha = roughness * roughness;
    float D = GGX_Distribution::D(wh, shading_ctx->N, alpha);
    float G = GGX_Distribution::G(wi, wo, shading_ctx->N, alpha);
    
    ColorRGB f = (G * D) * F / (4.f * dot(shading_ctx->N, wo) * dot(shading_ctx->N, wi));
    return f;
}

//
// Plastic BRDF
//
Plastic_BRDF::Plastic_BRDF(const Scene_Context& scene_ctx, const Shading_Context& shading_ctx, const Plastic_Material& params)
    : BSDF(shading_ctx)
{
    this->scene_ctx = &scene_ctx;
    this->shading_ctx = &shading_ctx;

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
