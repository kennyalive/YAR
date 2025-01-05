#include "std.h"
#include "lib/common.h"
#include "bsdf.h"

#include "bsdf_pbrt.h"
#include "parameter_evaluation.h"
#include "sampling.h"
#include "scattering.h"
#include "shading_context.h"
#include "scene_context.h"
#include "thread_context.h"

#include "lib/math.h"

BSDF::BSDF(const Scene_Context& scene_context, const Shading_Context& shading_context)
    : scene_context(scene_context)
    , normal(shading_context.normal)
{
    bitangent = cross(normal, shading_context.dpdu_shading).normalized();
    tangent = cross(bitangent, normal);
}

Vector3 BSDF::local_to_world(const Vector3& local_direction) const
{
    return Vector3{
        tangent.x * local_direction.x + bitangent.x * local_direction.y + normal.x * local_direction.z,
        tangent.y * local_direction.x + bitangent.y * local_direction.y + normal.y * local_direction.z,
        tangent.z * local_direction.x + bitangent.z * local_direction.y + normal.z * local_direction.z
    };
}

Vector3 BSDF::world_to_local(const Vector3& world_direction) const
{
    return Vector3 { 
        dot(world_direction, tangent),
        dot(world_direction, bitangent),
        dot(world_direction, normal)
    };
}

Vector3 BSDF::sample_microfacet_normal(Vector2 u, const Vector3& wo, float alpha) const
{
    Vector3 wh_local;
    if (ggx_sample_visible_normals) {
        Vector3 wo_local = world_to_local(wo);
        wh_local = GGX_sample_visible_microfacet_normal(u, wo_local, alpha, alpha);
    }
    else {
        wh_local = GGX_sample_microfacet_normal(u, alpha);
    }
    Vector3 wh = local_to_world(wh_local);
    ASSERT(dot(wh, normal) >= 0.f);
    return wh;
}

//
// Diffuse BRDF
//
Diffuse_BRDF::Diffuse_BRDF(const Thread_Context& thread_ctx, const Diffuse_Material& material)
    : BSDF(thread_ctx.scene_context, thread_ctx.shading_context)
{
    reflection_scattering = true;
    reflectance = evaluate_rgb_parameter(thread_ctx, material.reflectance);
}

ColorRGB Diffuse_BRDF::evaluate(const Vector3& /*wo*/, const Vector3& /*wi*/) const
{
    return Pi_Inv * reflectance;
}

ColorRGB Diffuse_BRDF::sample(Vector2 u, float u_scattering_type, const Vector3& wo, Vector3* wi, float* pdf) const
{
    Vector3 local_dir = sample_hemisphere_cosine(u);
    *wi = local_to_world(local_dir);
    *pdf = Diffuse_BRDF::pdf(wo, *wi);
    return Diffuse_BRDF::evaluate(wo, *wi);
}

float Diffuse_BRDF::pdf(const Vector3& /*wo*/, const Vector3& wi) const
{
    ASSERT(dot(normal, wi) >= 0.f);
    return dot(normal, wi) / Pi; // pdf for cosine-weighted hemisphere sampling
}

//
// Diffuse Transmission BSDF
//
Diffuse_Transmission_BSDF::Diffuse_Transmission_BSDF(const Thread_Context& thread_ctx,
    const Diffuse_Transmission_Material& material)
    : BSDF(thread_ctx.scene_context, thread_ctx.shading_context)
{
    reflection_scattering = true;
    transmission_scattering = true;

    ColorRGB scale = evaluate_rgb_parameter(thread_ctx, material.scale);

    reflectance = scale * evaluate_rgb_parameter(thread_ctx, material.reflectance);
    reflectance.clamp_to_unit_range();

    transmittance = scale * evaluate_rgb_parameter(thread_ctx, material.transmittance);
    transmittance.clamp_to_unit_range();
}

ColorRGB Diffuse_Transmission_BSDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    bool same_hemisphere = dot(wo, normal) * dot(wi, normal) > 0.f;
    if (same_hemisphere)
        return Pi_Inv * reflectance;
    else
        return Pi_Inv * transmittance;
}

ColorRGB Diffuse_Transmission_BSDF::sample(Vector2 u, float u_scattering_type, const Vector3& wo, Vector3* wi, float* pdf) const
{
    float max_r = reflectance.max_component_value();
    float max_t = transmittance.max_component_value();
    float p = max_r / (max_r + max_t);

    float sign = (u_scattering_type < p) ? 1.f : -1.f; // reflection or transmission
    Vector3 local_dir = sign * sample_hemisphere_cosine(u);

    *wi = local_to_world(local_dir);
    *pdf = Diffuse_Transmission_BSDF::pdf(wo, *wi);
    return Diffuse_Transmission_BSDF::evaluate(wo, *wi);
}

float Diffuse_Transmission_BSDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    float max_r = reflectance.max_component_value();
    float max_t = transmittance.max_component_value();

    float cos_theta = std::abs(dot(normal, wi));
    float pdf = cosine_hemisphere_pdf(cos_theta);

    bool same_hemisphere = dot(wo, normal) * dot(wi, normal) > 0.f;
    if (same_hemisphere)
        return (max_r / (max_r + max_t)) * pdf;
    else
        return (max_t / (max_r + max_t)) * pdf;
}

//
// Metal BRDF
//
Metal_BRDF::Metal_BRDF(const Thread_Context& thread_ctx, const Metal_Material& material)
    : BSDF(thread_ctx.scene_context, thread_ctx.shading_context)
{
    reflection_scattering = true;
    const float roughness = evaluate_float_parameter(thread_ctx, material.roughness);
    alpha = GGX_Distribution::roughness_to_alpha(thread_ctx, roughness, material.roughness_is_alpha);
    eta_i = evaluate_float_parameter(thread_ctx, material.eta_i);
    eta_t = evaluate_rgb_parameter(thread_ctx, material.eta);
    k_t = evaluate_rgb_parameter(thread_ctx, material.k);
}

ColorRGB Metal_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = conductor_fresnel(cos_theta_i, eta_i, eta_t, k_t);
    float G = GGX_Distribution::G(wi, wo, normal, alpha);
    float D = GGX_Distribution::D(wh, normal, alpha);
    float wo_dot_n = dot(wo, normal);
    float wi_dot_n = dot(wi, normal);

    ColorRGB f = microfacet_reflection(F, G, D, wo_dot_n, wi_dot_n);
    return f;
}

ColorRGB Metal_BRDF::sample(Vector2 u, float u_scattering_type, const Vector3& wo, Vector3* wi, float* pdf) const
{
    Vector3 wh = sample_microfacet_normal(u, wo, alpha);
    *wi = reflect(wo, wh);

    if (dot(normal, *wi) <= 0.f)
        return Color_Black;

    *pdf = microfacet_reflection_wi_pdf(wo, wh, normal, alpha);
    return evaluate(wo, *wi);
}

float Metal_BRDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    ASSERT(dot(normal, wi) >= 0.f);
    Vector3 wh = (wo + wi).normalized();
    return microfacet_reflection_wi_pdf(wo, wh, normal, alpha);
}

//
// Plastic BRDF
//
Plastic_BRDF::Plastic_BRDF(const Thread_Context& thread_ctx, const Plastic_Material& params)
    : BSDF(thread_ctx.scene_context, thread_ctx.shading_context)
{
    reflection_scattering = true;
    const float roughness = evaluate_float_parameter(thread_ctx, params.roughness);
    alpha = GGX_Distribution::roughness_to_alpha(thread_ctx, roughness, params.roughness_is_alpha);
    r0 = evaluate_float_parameter(thread_ctx, params.r0);
    diffuse_reflectance = evaluate_rgb_parameter(thread_ctx, params.diffuse_reflectance);
}

ColorRGB Plastic_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = schlick_fresnel(ColorRGB(0.04f), cos_theta_i);
    float G = GGX_Distribution::G(wi, wo, normal, alpha);
    float D = GGX_Distribution::D(wh, normal, alpha);
    float wo_dot_n = dot(wo, normal);
    float wi_dot_n = dot(wi, normal);

    ColorRGB base_specular = microfacet_reflection(F, G, D, wo_dot_n, wi_dot_n);
    ColorRGB specular = r0 * base_specular;

    ColorRGB diffuse = diffuse_reflectance * Pi_Inv;
    return diffuse + specular;
}

ColorRGB Plastic_BRDF::sample(Vector2 u, float u_scattering_type, const Vector3& wo, Vector3* wi, float* pdf) const
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
    *pdf = Plastic_BRDF::pdf(wo, *wi);
    return evaluate(wo, *wi);
}

float Plastic_BRDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    ASSERT(dot(normal, wi) >= 0.f);
    float diffuse_pdf = dot(normal, wi) / Pi;

    Vector3 wh = (wo + wi).normalized();
    float spec_pdf = microfacet_reflection_wi_pdf(wo, wh, normal, alpha);

    float pdf = 0.5f * (diffuse_pdf + spec_pdf);
    return pdf;
}

//
// Rough glass BSDF
//
Rough_Glass_BSDF::Rough_Glass_BSDF(const Thread_Context& thread_ctx, const Glass_Material& params)
    : BSDF(thread_ctx.scene_context, thread_ctx.shading_context)
{
    reflection_scattering = true;
    transmission_scattering = true;

    reflectance = evaluate_rgb_parameter(thread_ctx, params.reflectance);
    transmittance = evaluate_rgb_parameter(thread_ctx, params.transmittance);

    const float roughness = evaluate_float_parameter(thread_ctx, params.roughness);
    alpha = GGX_Distribution::roughness_to_alpha(thread_ctx, roughness, params.roughness_is_alpha);

    bool enter_event = thread_ctx.shading_context.nested_dielectric ?
        thread_ctx.current_dielectric_material == Null_Material :
        !thread_ctx.shading_context.original_shading_normal_was_flipped;

    float dielectric_ior = evaluate_float_parameter(thread_ctx, params.index_of_refraction);
    if (enter_event) {
        eta_o = 1.f;
        eta_i = dielectric_ior;
    }
    else {
        eta_o = dielectric_ior;
        eta_i = 1.f;
    }
}

ColorRGB Rough_Glass_BSDF::evaluate(const Vector3& wo, const Vector3& wi) const
{
    bool same_hemisphere = dot(wo, normal) * dot(wi, normal) > 0.f;
    if (same_hemisphere) { // reflection
        Vector3 wh = (wo + wi).normalized();
        float cos_theta_i = dot(wi, wh);
        float F = dielectric_fresnel(cos_theta_i, eta_i / eta_o);
        float G = GGX_Distribution::G(wi, wo, normal, alpha);
        float D = GGX_Distribution::D(wh, normal, alpha);
        float wo_dot_n = dot(wo, normal);
        float wi_dot_n = dot(wi, normal);

        float base_reflection = microfacet_reflection(F, G, D, wo_dot_n, wi_dot_n);
        ColorRGB f = reflectance * base_reflection;
        return f;
    }
    else { // transmission
        Vector3 wh = refraction_half_direction(eta_o, wo, eta_i, wi, normal);
        float wo_dot_wh = dot(wo, wh);
        float wi_dot_wh = dot(wi, wh);
        if (wo_dot_wh * wi_dot_wh > 0.f) {
            // The provided wo/wi directions can't form a refraction configuration.
            // When refraction is possible, then wo/wi directions should be in the
            // different hemispheres of the half-direction vector.
            return Color_Black;
        }

        float cos_theta_i = dot(wi, wh);
        float F = dielectric_fresnel(cos_theta_i, eta_o / eta_i);
        if (F == 1.f) {
            return Color_Black;
        }
        float G = GGX_Distribution::G(wi, wo, normal, alpha);
        float D = GGX_Distribution::D(wh, normal, alpha);
        float wo_dot_n = dot(wo, normal);
        float wi_dot_n = dot(wi, normal);
        

        float base_transmission = microfacet_transmission(F, G, D, wo_dot_n, wi_dot_n, wo_dot_wh, wi_dot_wh, eta_o, eta_i);
        ColorRGB f = transmittance * base_transmission;
        return f;
    }
}

ColorRGB Rough_Glass_BSDF::sample(Vector2 u, float u_scattering_type, const Vector3& wo, Vector3* wi, float* pdf) const
{
    Vector3 wh = sample_microfacet_normal(u, wo, alpha);
    Vector3 reflection_wi = reflect(wo, wh);
    if (dot(reflection_wi, normal) <= 0.f) {
        return Color_Black;
    }

    float max_r = reflectance.max_component_value();
    float max_t = transmittance.max_component_value();
    float reflection_ratio = max_r / (max_r + max_t);

    float cos_theta_i = dot(wo, wh);
    ASSERT(cos_theta_i > 0.f);

    float fresnel = dielectric_fresnel(cos_theta_i, eta_i / eta_o);
    float r = fresnel * reflection_ratio;
    float t = (1.f - fresnel) * (1 - reflection_ratio);
    float reflection_probability = ((r + t) == 0) ? 0.f : r / (r + t);

    if (u_scattering_type < reflection_probability) {
        *wi = reflection_wi;
    }
    else {
        bool refracted = refract(wo, wh, eta_o / eta_i, wi);
        if (!refracted) {
            return Color_Black;
        }
        if (dot(*wi, normal) >= 0.f) {
            return Color_Black;
        }
    }
    *pdf = Rough_Glass_BSDF::pdf(wo, *wi);
    if (*pdf == 0.f) {
        return Color_Black;
    }
    return Rough_Glass_BSDF::evaluate(wo, *wi);
}

float Rough_Glass_BSDF::pdf(const Vector3& wo, const Vector3& wi) const
{
    float max_r = reflectance.max_component_value();
    float max_t = transmittance.max_component_value();
    float reflection_ratio = max_r / (max_r + max_t);

    bool same_hemisphere = dot(wo, normal) * dot(wi, normal) > 0.f;
    if (same_hemisphere) { // reflection
        Vector3 wh = (wo + wi).normalized();
        float reflection_pdf = microfacet_reflection_wi_pdf(wo, wh, normal, alpha);
        float cos_theta_i = dot(wi, wh);

        float fresnel = dielectric_fresnel(cos_theta_i, eta_i / eta_o);
        float r = fresnel * reflection_ratio;
        float t = (1.f - fresnel) * (1 - reflection_ratio);
        float reflection_probability = ((r + t) == 0) ? 0.f : r / (r + t);

        float pdf = reflection_pdf * reflection_probability;
        return pdf;
    }
    else { // transmission
        Vector3 wh = refraction_half_direction(eta_o, wo, eta_i, wi, normal);
        float wo_dot_wh = dot(wo, wh);
        float wi_dot_wh = dot(wi, wh);
        if (wo_dot_wh * wi_dot_wh > 0.f) {
            // The provided wo/wi directions can't form a refraction configuration.
            // When refraction is possible, then wo/wi directions should be in the
            // different hemispheres of the half-direction vector.
            return 0.f;
        }

        float cos_theta = dot(wo, wh);
        float fresnel = dielectric_fresnel(cos_theta, eta_i / eta_o);
        float r = fresnel * reflection_ratio;
        float t = (1.f - fresnel) * (1 - reflection_ratio);
        float reflection_probability = ((r + t) == 0) ? 0.f : r / (r + t);

        float transmission_pdf = microfacet_transmission_wi_pdf(wo, wi, wh, normal, alpha, eta_o, eta_i);
        float pdf = transmission_pdf * (1.f - reflection_probability);
        return pdf;
    }
}

//
// Ashikhmin_Shirley_Phong_BRDF
//
// BRDF described in "An Anisotropic Phong Light Reflection Model", Michael Ashikhmin, Peter Shirley.
// https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.18.4504&rep=rep1&type=pdf
//
Ashikhmin_Shirley_Phong_BRDF::Ashikhmin_Shirley_Phong_BRDF(const Thread_Context& thread_ctx, const Coated_Diffuse_Material& params)
    : BSDF(thread_ctx.scene_context, thread_ctx.shading_context)
{
    reflection_scattering = true;
    const float roughness = evaluate_float_parameter(thread_ctx, params.roughness);
    alpha = GGX_Distribution::roughness_to_alpha(thread_ctx, roughness, params.roughness_is_alpha);
    r0 = evaluate_rgb_parameter(thread_ctx, params.r0);
    diffuse_reflectance = evaluate_rgb_parameter(thread_ctx, params.diffuse_reflectance);
}

ColorRGB Ashikhmin_Shirley_Phong_BRDF::evaluate(const Vector3& wo, const Vector3& wi) const {
    Vector3 wh = (wo + wi).normalized();

    float cos_theta_i = dot(wi, wh);
    ASSERT(cos_theta_i >= 0);

    ColorRGB F = schlick_fresnel(r0, cos_theta_i);
    float D = GGX_Distribution::D(wh, normal, alpha);

    ColorRGB specular_brdf = F * (D / (4.f * cos_theta_i * std::max(dot(normal, wo), dot(normal, wi))));

    auto pow5 = [](float v) { return (v * v) * (v * v) * v; };

    ColorRGB diffuse_brdf =
            (diffuse_reflectance * (ColorRGB(1.f) - r0)) *
            (28.f / (23.f*Pi) * (1.f - pow5(1.f - 0.5f * dot(normal, wi))) * (1.f - pow5(1.f - 0.5f * dot(normal, wo))));

    return diffuse_brdf + specular_brdf;
}

ColorRGB Ashikhmin_Shirley_Phong_BRDF::sample(Vector2 u, float u_scattering_type, const Vector3& wo, Vector3* wi, float* pdf) const {
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
    *pdf = Ashikhmin_Shirley_Phong_BRDF::pdf(wo, *wi);
    return evaluate(wo, *wi);
}

float Ashikhmin_Shirley_Phong_BRDF::pdf(const Vector3& wo, const Vector3& wi) const {
    ASSERT(dot(normal, wi) >= 0.f);
    float diffuse_pdf = dot(normal, wi) / Pi;

    Vector3 wh = (wo + wi).normalized();
    float spec_pdf = microfacet_reflection_wi_pdf(wo, wh, normal, alpha);

    float pdf = 0.5f * (diffuse_pdf + spec_pdf);
    return pdf;
}

const BSDF* create_bsdf(Thread_Context& thread_ctx, Material_Handle material) {
    const Scene_Context& scene_ctx = thread_ctx.scene_context;
    Shading_Context& shading_ctx = thread_ctx.shading_context;
    switch (material.type) {
    case Material_Type::diffuse:
    {
        const Diffuse_Material& params = scene_ctx.materials.diffuse[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Diffuse_BRDF>();
        return new (bsdf_allocation) Diffuse_BRDF(thread_ctx, params);
    }
    case Material_Type::diffuse_transmission:
    {
        const Diffuse_Transmission_Material& params = scene_ctx.materials.diffuse_transmission[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Diffuse_Transmission_BSDF>();
        return new (bsdf_allocation) Diffuse_Transmission_BSDF(thread_ctx, params);
    }
    case Material_Type::metal:
    {
        const Metal_Material& params = scene_ctx.materials.metal[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Metal_BRDF>();
        return new (bsdf_allocation) Metal_BRDF(thread_ctx, params);
    }
    case Material_Type::plastic:
    {
        const Plastic_Material& params = scene_ctx.materials.plastic[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        if (thread_ctx.scene_context.pbrt3_scene) {
            void* bsdf_allocation = thread_ctx.memory_pool.allocate<Pbrt3_Plastic_BRDF>();
            return new (bsdf_allocation) Pbrt3_Plastic_BRDF(thread_ctx, params);
        }
        else {
            void* bsdf_allocation = thread_ctx.memory_pool.allocate<Plastic_BRDF>();
            return new (bsdf_allocation) Plastic_BRDF(thread_ctx, params);
        }
    }
    case Material_Type::coated_diffuse:
    {
        const Coated_Diffuse_Material& params = scene_ctx.materials.coated_diffuse[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Ashikhmin_Shirley_Phong_BRDF>();
        return new (bsdf_allocation) Ashikhmin_Shirley_Phong_BRDF(thread_ctx, params);
    }
    case Material_Type::glass:
    {
        const Glass_Material& params = scene_ctx.materials.glass[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Rough_Glass_BSDF>();
        return new (bsdf_allocation) Rough_Glass_BSDF(thread_ctx, params);
    }
    case Material_Type::pbrt3_uber:
    {
        const Pbrt3_Uber_Material& params = scene_ctx.materials.pbrt3_uber[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Pbrt3_Uber_BRDF>();
        return new (bsdf_allocation) Pbrt3_Uber_BRDF(thread_ctx, params);
    }
    case Material_Type::pbrt3_translucent:
    {
        const Pbrt3_Translucent_Material& params = scene_ctx.materials.pbrt3_translucent[material.index];
        shading_ctx.apply_bump_map(scene_ctx, params.bump_map);
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Pbrt3_Translucent_BSDF>();
        return new (bsdf_allocation) Pbrt3_Translucent_BSDF(thread_ctx, params);
    }
    case Material_Type::pbrt3_fourier:
    {
        const Pbrt3_Fourier_Material& params = scene_ctx.materials.pbrt3_fourier[material.index];
        void* bsdf_allocation = thread_ctx.memory_pool.allocate<Pbrt3_Fourier_BSDF>();
        return new (bsdf_allocation) Pbrt3_Fourier_BSDF(thread_ctx, params);
    }
    default:
    {
        ASSERT(false);
        return nullptr;
    }
    }
}
