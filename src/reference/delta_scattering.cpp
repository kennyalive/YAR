#include "std.h"
#include "lib/common.h"
#include "delta_scattering.h"

#include "parameter_evaluation.h"
#include "scattering.h"
#include "scene_context.h"
#include "shading_context.h"
#include "thread_context.h"

namespace {
enum class Delta_Scattering_Type {
    none,
    reflection,
    transmission,
    passthrough
};

struct Delta_Info {
    Delta_Scattering_Type scattering_type = Delta_Scattering_Type::none;
    ColorRGB attenuation = Color_White;
    float etaI_over_etaT = 0.f; // used only by Delta_Scattering_Type::transmission
    float delta_layer_selection_probability = 0.f;
};
}

static Delta_Info get_perfect_reflector_info(Thread_Context& thread_ctx,
    const Perfect_Reflector_Material& params)
{
    Delta_Info result;
    result.scattering_type = Delta_Scattering_Type::reflection;
    result.attenuation = evaluate_rgb_parameter(thread_ctx, params.reflectance);
    result.delta_layer_selection_probability = 1.f;
    return result;
}

static Delta_Info get_perfect_refractor_info(Thread_Context& thread_ctx, const Perfect_Refractor_Material& params)
{
    const Shading_Context& shading_ctx = thread_ctx.shading_context;

    bool enter_event = shading_ctx.nested_dielectric ?
        thread_ctx.current_dielectric_material == Null_Material :
        !shading_ctx.original_shading_normal_was_flipped;

    float dielectric_ior = evaluate_float_parameter(thread_ctx, params.index_of_refraction);

    Delta_Info result;
    result.scattering_type = Delta_Scattering_Type::transmission;
    result.etaI_over_etaT = enter_event ? 1.f / dielectric_ior : dielectric_ior;
    result.delta_layer_selection_probability = 1.f;
    return result;
}

static Delta_Info get_glass_info(Thread_Context& thread_ctx, const Glass_Material& params, float u)
{
    float roughness = evaluate_float_parameter(thread_ctx, params.roughness);
    if (roughness > 0.f) {
        return Delta_Info{}; // it's not a delta scattering, should be handled by Rough_Glass_BSDF
    }

    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    Delta_Info result;

    bool enter_event = shading_ctx.nested_dielectric ?
        thread_ctx.current_dielectric_material == Null_Material :
        !shading_ctx.original_shading_normal_was_flipped;

    float dielectric_ior = evaluate_float_parameter(thread_ctx, params.index_of_refraction);
    float etaT_over_etaI = enter_event ? dielectric_ior : 1.f / dielectric_ior;

    float cos_theta_i = dot(shading_ctx.normal, shading_ctx.wo); // dot(n, wi) == dot(n, wo)
    ASSERT(cos_theta_i > 0.f);
    float fresnel = dielectric_fresnel(cos_theta_i, etaT_over_etaI);

    // NOTE: for total internal reflection 'fresnel == 1', and the following condition
    // 'r < fresnel' is always true. This gives a guarantee that transmission event is
    // never selected in the case of total internal reflection.

    if (u < fresnel) {
        result.scattering_type = Delta_Scattering_Type::reflection;
        // The reflection event is choosen with probability = fresnel.
        // attenuation = fresnel * reflectance / probability => attenuation = reflectance.
        result.attenuation = evaluate_rgb_parameter(thread_ctx, params.reflectance);
    }
    else {
        float etaI_over_etaT = 1.f / etaT_over_etaI;

        result.scattering_type = Delta_Scattering_Type::transmission;

        // The transmission event is choosen with probability = 1 - fresnel.
        // attenuation = (1 - fresnel) * transmittance / probability => attenuation = transmittance.
        result.attenuation = evaluate_rgb_parameter(thread_ctx, params.transmittance);

        // Radiance scaling due to IoR boundary.
        result.attenuation *= etaI_over_etaT * etaI_over_etaT;

        result.etaI_over_etaT = etaI_over_etaT;
    }
    result.delta_layer_selection_probability = 1.f;
    return result;
}

static Delta_Info get_pbrt_uber_info(Thread_Context& thread_ctx, const Pbrt3_Uber_Material& params, float* u_scattering_type)
{
    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    Delta_Info result;

    float fp_index = *u_scattering_type * float(params.component_count);
    int component_index = int(fp_index);
    ASSERT(component_index < params.component_count);
    int component_type = params.components[component_index];

    // re-normalize u_scattering_type random variable, so it can be re-used in the bsdf pipeline
    *u_scattering_type = std::clamp(fp_index - float(component_index), 0.f, One_Minus_Epsilon);

    if (component_type == Pbrt3_Uber_Material::DELTA_REFLECTION) {
        bool enter_event = shading_ctx.nested_dielectric ?
            thread_ctx.current_dielectric_material == Null_Material :
            !shading_ctx.original_shading_normal_was_flipped;

        float dielectric_ior = evaluate_float_parameter(thread_ctx, params.index_of_refraction);
        float etaT_over_etaI = enter_event ? dielectric_ior : 1.f / dielectric_ior;

        float cos_theta_i = dot(shading_ctx.normal, shading_ctx.wo); // dot(n, wi) == dot(n, wo)
        ASSERT(cos_theta_i > 0.f);
        float fresnel = dielectric_fresnel(cos_theta_i, etaT_over_etaI);

        ColorRGB reflectance = evaluate_rgb_parameter(thread_ctx, params.delta_reflectance);

        result.scattering_type = Delta_Scattering_Type::reflection;
        result.attenuation = (float(params.component_count) * fresnel) * reflectance;
    }
    else if (component_type == Pbrt3_Uber_Material::DELTA_TRANSMISSION) {
        ASSERT(false); // TODO: not implemented
    }
    else if (component_type == Pbrt3_Uber_Material::OPACITY) {
        ColorRGB opacity = evaluate_rgb_parameter(thread_ctx, params.opacity);
        ASSERT(opacity.r <= 1.f && opacity.g <= 1.f && opacity.b <= 1.f);
        result.scattering_type = Delta_Scattering_Type::passthrough;
        result.attenuation = float(params.component_count) * (Color_White - opacity);
    }

    // Compute delta layer selection probability.
    float delta_terms_count = 0.f;
    for (int i = 0; i < params.component_count; i++) {
        if (params.components[i] >= Pbrt3_Uber_Material::DELTA_REFLECTION)
            delta_terms_count += 1.f;
    }
    result.delta_layer_selection_probability = delta_terms_count / float(params.component_count);
    return result;
}

bool check_for_delta_scattering_event(Thread_Context& thread_ctx, float* u_scattering_type, Delta_Scattering* delta_scattering)
{
    const Scene_Context& scene_ctx = thread_ctx.scene_context;
    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    const Material_Handle material = shading_ctx.material;

    Delta_Info delta_info;
    if (material.type == Material_Type::perfect_reflector) {
        const Perfect_Reflector_Material& params = scene_ctx.materials.perfect_reflector[material.index];
        delta_info = get_perfect_reflector_info(thread_ctx, params);
    }
    else if (material.type == Material_Type::perfect_refractor) {
        const Perfect_Refractor_Material& params = scene_ctx.materials.perfect_refractor[material.index];
        delta_info = get_perfect_refractor_info(thread_ctx, params);
    }
    else if (material.type == Material_Type::glass) {
        const Glass_Material& params = scene_ctx.materials.glass[material.index];
        delta_info = get_glass_info(thread_ctx, params, *u_scattering_type);
    }
    else if (material.type == Material_Type::pbrt3_uber) {
        const Pbrt3_Uber_Material& params = scene_ctx.materials.pbrt3_uber[material.index];
        delta_info = get_pbrt_uber_info(thread_ctx, params, u_scattering_type);
    }

    delta_scattering->delta_layer_selection_probability = delta_info.delta_layer_selection_probability;

    if (delta_info.scattering_type == Delta_Scattering_Type::none)
        return false;

    // Update current dielectric state.
    if (shading_ctx.nested_dielectric && delta_info.scattering_type == Delta_Scattering_Type::transmission)
    {
        if (thread_ctx.current_dielectric_material == Null_Material)
            thread_ctx.current_dielectric_material = material;
        else {
            ASSERT(thread_ctx.current_dielectric_material == material);
            thread_ctx.current_dielectric_material = Null_Material;
        }
    }

    //
    // Compute new ray direction.
    //
    const Path_Context& path_ctx = thread_ctx.path_context;
    const Raytracer_Config& rt_config = scene_ctx.raytracer_config;

    Vector3 delta_direction;
    Differential_Rays differential_rays;
    bool has_differential_rays = false;

    if (delta_info.scattering_type == Delta_Scattering_Type::reflection) {
        delta_direction = reflect(shading_ctx.wo, shading_ctx.normal);

        has_differential_rays =
            shading_ctx.has_dxdy_derivatives &&
            path_ctx.bounce_count < rt_config.max_differential_ray_specular_bounces;

        if (has_differential_rays) {
            Ray reflected_ray{ shading_ctx.position, delta_direction };
            differential_rays = shading_ctx.compute_differential_rays_for_specular_reflection(reflected_ray);
        }
    }
    else if (delta_info.scattering_type == Delta_Scattering_Type::transmission) {
        bool refracted = refract(shading_ctx.wo, shading_ctx.normal, delta_info.etaI_over_etaT, &delta_direction);
        // Total internal reflection can not happen if we have reached this code path.
        // The comments earlier in this file explain why we have this guarantee.
        ASSERT(refracted);

        has_differential_rays =
            shading_ctx.has_dxdy_derivatives &&
            path_ctx.bounce_count < rt_config.max_differential_ray_specular_bounces;

        if (has_differential_rays) {
            Ray transmitted_ray{ shading_ctx.position, delta_direction };
            differential_rays = shading_ctx.compute_differential_rays_for_specular_transmission(
                transmitted_ray, delta_info.etaI_over_etaT);
        }
    }
    else {
        ASSERT(delta_info.scattering_type == Delta_Scattering_Type::passthrough);
        delta_direction = -shading_ctx.wo;
    }

    delta_scattering->attenuation = delta_info.attenuation;
    delta_scattering->delta_direction = delta_direction;
    delta_scattering->has_differential_rays = has_differential_rays;
    delta_scattering->differential_rays = differential_rays;
    return true;
}
