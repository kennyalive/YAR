#include "std.h"
#include "lib/common.h"
#include "specular_scattering.h"

#include "context.h"
#include "parameter_evaluation.h"
#include "scattering.h"
#include "shading_context.h"

static Auxilary_Rays specularly_reflect_auxilary_rays(const Shading_Context& shading_ctx, const Ray& reflected_ray)
{
    const Vector3& wo = shading_ctx.wo;
    const Vector3& n = shading_ctx.normal;

    Vector3 dndx = shading_ctx.dndu * shading_ctx.dudx + shading_ctx.dndv * shading_ctx.dvdx;
    float d_wo_dot_n_dx = dot(shading_ctx.dwo_dx, n) + dot(wo, dndx);
    Vector3 dwi_dx = 2.f * (d_wo_dot_n_dx * n + dot(wo, n) * dndx) - shading_ctx.dwo_dx;

    Vector3 dndy = shading_ctx.dndu * shading_ctx.dudy + shading_ctx.dndv * shading_ctx.dvdy;
    float d_wo_dot_n_dy = dot(shading_ctx.dwo_dy, n) + dot(wo, dndy);
    Vector3 dwi_dy = 2.f * (d_wo_dot_n_dy * n + dot(wo, n) * dndy) - shading_ctx.dwo_dy;

    Auxilary_Rays reflected_auxilary_rays;
    reflected_auxilary_rays.ray_dx_offset.origin = reflected_ray.origin + shading_ctx.dpdx;
    reflected_auxilary_rays.ray_dx_offset.direction = (reflected_ray.direction + dwi_dx).normalized();
    reflected_auxilary_rays.ray_dy_offset.origin = reflected_ray.origin + shading_ctx.dpdy;
    reflected_auxilary_rays.ray_dy_offset.direction = (reflected_ray.direction + dwi_dy).normalized();
    return reflected_auxilary_rays;
}

static Auxilary_Rays specularly_transmit_auxilary_rays(const Shading_Context& shading_ctx, const Ray& transmitted_ray, float etaI_over_etaT)
{
    const Vector3& wo = shading_ctx.wo;
    const Vector3& n = shading_ctx.normal;
    const float eta = etaI_over_etaT;

    float cos_o = dot(wo, n);
    ASSERT(cos_o > 0.f);
    float cos_t = -dot(transmitted_ray.direction, n);
    ASSERT(cos_t > 0.f);

    Vector3 dndx = shading_ctx.dndu * shading_ctx.dudx + shading_ctx.dndv * shading_ctx.dvdx;
    float d_wo_dot_n_dx = dot(shading_ctx.dwo_dx, n) + dot(wo, dndx);
    float d_cos_t_dx = eta * eta * cos_o * d_wo_dot_n_dx / cos_t;
    Vector3 dwi_dx = -eta * shading_ctx.dwo_dx + (eta * cos_o - cos_t) * dndx + (eta * d_wo_dot_n_dx - d_cos_t_dx) * n;

    Vector3 dndy = shading_ctx.dndu * shading_ctx.dudy + shading_ctx.dndv * shading_ctx.dvdy;
    float d_wo_dot_n_dy = dot(shading_ctx.dwo_dy, n) + dot(wo, dndy);
    float d_cos_t_dy = eta * eta * cos_o * d_wo_dot_n_dy / cos_t;
    Vector3 dwi_dy = -eta * shading_ctx.dwo_dy + (eta * cos_o - cos_t) * dndy + (eta * d_wo_dot_n_dy - d_cos_t_dy) * n;

    Auxilary_Rays transmitted_auxilary_rays;
    transmitted_auxilary_rays.ray_dx_offset.origin = transmitted_ray.origin + shading_ctx.dpdx;
    transmitted_auxilary_rays.ray_dx_offset.direction = (transmitted_ray.direction + dwi_dx).normalized();
    transmitted_auxilary_rays.ray_dy_offset.origin = transmitted_ray.origin + shading_ctx.dpdy;
    transmitted_auxilary_rays.ray_dy_offset.direction = (transmitted_ray.direction + dwi_dy).normalized();
    return transmitted_auxilary_rays;
}

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

    const int max_differential_ray_specular_bounces = thread_ctx.scene_context->scene->raytracer_config.max_differential_ray_specular_bounces;
    int differential_ray_bounce_count = 0;

    *specular_attenuation = Color_White;

    while (specular_scattering.type != Specular_Scattering_Type::none && path_ctx.bounce_count < max_bounces) {
        path_ctx.bounce_count++;
        path_ctx.perfect_specular_bounce_count++;

        differential_ray_bounce_count++;
        const bool compute_differential_rays =
            shading_ctx.has_auxilary_rays_data &&
            differential_ray_bounce_count <= max_differential_ray_specular_bounces;

        *specular_attenuation *= specular_scattering.scattering_coeff;

        Ray scattered_ray{ shading_ctx.position };
        Auxilary_Rays scattered_auxilary_rays;

        if (specular_scattering.type == Specular_Scattering_Type::specular_reflection) {
            scattered_ray.direction = reflect(shading_ctx.wo, shading_ctx.normal);

            if (compute_differential_rays)
                scattered_auxilary_rays = specularly_reflect_auxilary_rays(shading_ctx, scattered_ray);
        }
        else {
            ASSERT(specular_scattering.type == Specular_Scattering_Type::specular_transmission);
            const float eta = specular_scattering.etaI_over_etaT;

            const bool refracted = refract(shading_ctx.wo, shading_ctx.normal, eta, &scattered_ray.direction);
            ASSERT(refracted); // specular_transmission event should never be selected when total internal reflection happens

            if (compute_differential_rays)
                scattered_auxilary_rays = specularly_transmit_auxilary_rays(shading_ctx, scattered_ray, eta);
        }

        if (!trace_ray(thread_ctx, scattered_ray, shading_ctx.has_auxilary_rays_data ? &scattered_auxilary_rays : nullptr))
            return false;
    }
    return true;
}
