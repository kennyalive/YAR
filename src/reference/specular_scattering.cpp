#include "std.h"
#include "lib/common.h"
#include "specular_scattering.h"

#include "context.h"
#include "parameter_evaluation.h"
#include "scattering.h"
#include "shading_context.h"

#include "lib/scene.h"

Specular_Scattering get_specular_scattering_params(Thread_Context& thread_ctx, Material_Handle material_handle)
{
    const Scene_Context& scene_ctx = *thread_ctx.scene_context;
    const Shading_Context& shading_ctx = thread_ctx.shading_context;

    Specular_Scattering specular_scattering{ Specular_Scattering_Type::none };

    if (material_handle.type == Material_Type::perfect_reflector) {
        const Perfect_Reflector_Material& params = scene_ctx.materials.perfect_reflector[material_handle.index];
        specular_scattering.type = Specular_Scattering_Type::specular_reflection;
        specular_scattering.scattering_coeff = evaluate_rgb_parameter(thread_ctx, params.reflectance);
    }
    else if (material_handle.type == Material_Type::perfect_refractor) {
        const Perfect_Refractor_Material& params = scene_ctx.materials.perfect_refractor[material_handle.index];
        specular_scattering.type = Specular_Scattering_Type::specular_transmission;

        float dielectric_ior = evaluate_float_parameter(thread_ctx, params.index_of_refraction);
        if (thread_ctx.current_dielectric_material == Null_Material) { // dielectric enter event
            thread_ctx.current_dielectric_material = material_handle;
            specular_scattering.etaI_over_etaT = 1.f / dielectric_ior;
        }
        else {  // dielectric exit event
            ASSERT(thread_ctx.current_dielectric_material == material_handle);
            thread_ctx.current_dielectric_material = Null_Material;
            specular_scattering.etaI_over_etaT = dielectric_ior / 1.f;
        }
    }
    else if (material_handle.type == Material_Type::glass) {
        const Glass_Material& params = scene_ctx.materials.glass[material_handle.index];
        float dielectric_ior = evaluate_float_parameter(thread_ctx, params.index_of_refraction);
        if (thread_ctx.current_dielectric_material == Null_Material) {
            specular_scattering.etaI_over_etaT = 1.f / dielectric_ior;
        }
        else {
            ASSERT(thread_ctx.current_dielectric_material == material_handle);
            specular_scattering.etaI_over_etaT = dielectric_ior / 1.f;
        }

        // fresnel depends on incident direction (Wi) but for specular reflection dot(n, wi) == dot(n, wo)
        float cos_theta_i = dot(shading_ctx.normal, shading_ctx.wo);
        ASSERT(cos_theta_i > 0.f);
        float fresnel = dielectric_fresnel(cos_theta_i, 1.f / specular_scattering.etaI_over_etaT);

        float r = thread_ctx.rng.get_float();
        if (r < fresnel) {
            specular_scattering.type = Specular_Scattering_Type::specular_reflection;
            // The reflection event is choosen with probability == fresnel.
            // coeff is computed as fresnel * reflectance / probability => coeff == reflectance.
            specular_scattering.scattering_coeff = evaluate_rgb_parameter(thread_ctx, params.reflectance);
        }
        else {
            specular_scattering.type = Specular_Scattering_Type::specular_transmission;
            // The transmission event is choosen with probability == 1 - fresnel.
            // coeff is computed as (1 - fresnel) * transmittance / probability => coeff = transmittance.
            specular_scattering.scattering_coeff = evaluate_rgb_parameter(thread_ctx, params.transmittance);

            specular_scattering.scattering_coeff *= specular_scattering.etaI_over_etaT * specular_scattering.etaI_over_etaT;

            // Update current dielectric state.
            if (thread_ctx.current_dielectric_material == Null_Material)
                thread_ctx.current_dielectric_material = material_handle;
            else
                thread_ctx.current_dielectric_material = Null_Material;
        }
    }
    return specular_scattering;
}

bool trace_specular_bounces(Thread_Context& thread_ctx, int max_bounces, ColorRGB* specular_attenuation)
{
    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    Path_Context& path_ctx = thread_ctx.path_context;

    const Specular_Scattering& specular_scattering = shading_ctx.specular_scattering;
    ASSERT(specular_scattering.type != Specular_Scattering_Type::none);

    const int max_differential_ray_bounces =
        thread_ctx.scene_context->raytracer_config.max_differential_ray_specular_bounces;

    *specular_attenuation = Color_White;
    while (specular_scattering.type != Specular_Scattering_Type::none && path_ctx.bounce_count < max_bounces) {
        path_ctx.bounce_count++;
        path_ctx.perfect_specular_bounce_count++;
        *specular_attenuation *= specular_scattering.scattering_coeff;

        const bool compute_differential_rays =
            shading_ctx.has_dxdy_derivatives &&
            path_ctx.bounce_count <= max_differential_ray_bounces;

        Ray ray{ shading_ctx.position }; // specularly reflected or transmitted ray
        Differential_Rays differential_rays;

        if (specular_scattering.type == Specular_Scattering_Type::specular_reflection) {
            ray.direction = reflect(shading_ctx.wo, shading_ctx.normal);
            if (compute_differential_rays)
                differential_rays = shading_ctx.compute_differential_rays_for_specular_reflection(ray);
        }
        else {
            ASSERT(specular_scattering.type == Specular_Scattering_Type::specular_transmission);
            const float eta = specular_scattering.etaI_over_etaT;
            const bool refracted = refract(shading_ctx.wo, shading_ctx.normal, eta, &ray.direction);
            ASSERT(refracted); // specular_transmission event should never be selected when total internal reflection happens
            if (compute_differential_rays)
                differential_rays = shading_ctx.compute_differential_rays_for_specular_transmission(ray, eta);
        }

        if (!trace_ray(thread_ctx, ray, compute_differential_rays ? &differential_rays : nullptr))
            return false;
    }
    return true;
}
