#include "std.h"
#include "lib/common.h"
#include "specular_scattering.h"

#include "context.h"
#include "parameter_evaluation.h"
#include "scattering.h"
#include "shading_context.h"

static void specularly_reflect_auxilary_rays(const Shading_Context& shading_ctx, const Ray& reflected_ray,
    Auxilary_Rays* auxilary_rays)
{
    if (!auxilary_rays)
        return;

    const Vector3& wo = shading_ctx.wo;
    const Vector3& n = shading_ctx.normal;

    Vector3 dndx = shading_ctx.dndu * shading_ctx.dudx + shading_ctx.dndv * shading_ctx.dvdx;
    Vector3 dwo_dx = (-auxilary_rays->ray_dx_offset.direction) - wo;
    float d_wo_dot_n_dx = dot(dwo_dx, n) + dot(wo, dndx);
    Vector3 dwi_dx = 2.f * (d_wo_dot_n_dx * n + dot(wo, n) * dndx) - dwo_dx;

    Vector3 dndy = shading_ctx.dndu * shading_ctx.dudy + shading_ctx.dndv * shading_ctx.dvdy;
    Vector3 dwo_dy = (-auxilary_rays->ray_dy_offset.direction) - wo;
    float d_wo_dot_n_dy = dot(dwo_dy, n) + dot(wo, dndy);
    Vector3 dwi_dy = 2.f * (d_wo_dot_n_dy * n + dot(wo, n) * dndy) - dwo_dy;

    Auxilary_Rays reflected_auxilary_rays;
    reflected_auxilary_rays.ray_dx_offset.origin = reflected_ray.origin + shading_ctx.dpdx;
    reflected_auxilary_rays.ray_dx_offset.direction = (reflected_ray.direction + dwi_dx).normalized();
    reflected_auxilary_rays.ray_dy_offset.origin = reflected_ray.origin + shading_ctx.dpdy;
    reflected_auxilary_rays.ray_dy_offset.direction = (reflected_ray.direction + dwi_dy).normalized();
    *auxilary_rays = reflected_auxilary_rays;
}

static void specularly_transmit_auxilary_rays(const Shading_Context& shading_ctx, const Ray& transmitted_ray,
    float etaI_over_etaT, Auxilary_Rays* auxilary_rays)
{
    if (!auxilary_rays)
        return;

    const Vector3& wo = shading_ctx.wo;
    const Vector3& n = shading_ctx.normal;
    const float eta = etaI_over_etaT;

    float cos_o = dot(wo, n);
    ASSERT(cos_o > 0.f);
    float cos_t = -dot(transmitted_ray.direction, n);
    ASSERT(cos_t > 0.f);

    Vector3 dndx = shading_ctx.dndu * shading_ctx.dudx + shading_ctx.dndv * shading_ctx.dvdx;
    Vector3 dwo_dx = (-auxilary_rays->ray_dx_offset.direction) - wo;
    float d_wo_dot_n_dx = dot(dwo_dx, n) + dot(wo, dndx);
    float d_cos_t_dx = eta * eta * cos_o * d_wo_dot_n_dx / cos_t;
    Vector3 dwi_dx = -eta * dwo_dx + (eta * cos_o - cos_t) * dndx + (eta * d_wo_dot_n_dx - d_cos_t_dx) * n;

    Vector3 dndy = shading_ctx.dndu * shading_ctx.dudy + shading_ctx.dndv * shading_ctx.dvdy;
    Vector3 dwo_dy = (-auxilary_rays->ray_dy_offset.direction) - wo;
    float d_wo_dot_n_dy = dot(dwo_dy, n) + dot(wo, dndy);
    float d_cos_t_dy = eta * eta * cos_o * d_wo_dot_n_dy / cos_t;
    Vector3 dwi_dy = -eta * dwo_dy + (eta * cos_o - cos_t) * dndy + (eta * d_wo_dot_n_dy - d_cos_t_dy) * n;

    Auxilary_Rays transmitted_auxilary_rays;
    transmitted_auxilary_rays.ray_dx_offset.origin = transmitted_ray.origin + shading_ctx.dpdx;
    transmitted_auxilary_rays.ray_dx_offset.direction = (transmitted_ray.direction + dwi_dx).normalized();
    transmitted_auxilary_rays.ray_dy_offset.origin = transmitted_ray.origin + shading_ctx.dpdy;
    transmitted_auxilary_rays.ray_dy_offset.direction = (transmitted_ray.direction + dwi_dy).normalized();
    *auxilary_rays = transmitted_auxilary_rays;
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

            // Update current dielectric state.
            if (thread_ctx.current_dielectric_material == Null_Material)
                thread_ctx.current_dielectric_material = material_handle;
            else
                thread_ctx.current_dielectric_material = Null_Material;
        }
    }
    return specular_scattering;
}

bool trace_specular_bounces(Thread_Context& thread_ctx, const Auxilary_Rays* incident_auxilary_rays,
    int max_specular_bounces, ColorRGB* specular_bounces_contribution)
{
    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    const Specular_Scattering& specular_scattering = shading_ctx.specular_scattering;
    ASSERT(specular_scattering.type != Specular_Scattering_Type::none);

    Auxilary_Rays scattered_auxilary_rays;
    Auxilary_Rays* p_scattered_auxilary_rays = nullptr; // either nullptr or points to scattered_auxilary_rays
    if (incident_auxilary_rays) {
        scattered_auxilary_rays = *incident_auxilary_rays;
        p_scattered_auxilary_rays = &scattered_auxilary_rays;
    }

    *specular_bounces_contribution = Color_White;

    while (specular_scattering.type != Specular_Scattering_Type::none && max_specular_bounces > 0) {
        max_specular_bounces--;
        const float eta = specular_scattering.etaI_over_etaT;

        Ray scattered_ray{ shading_ctx.position };
        if (specular_scattering.type == Specular_Scattering_Type::specular_reflection) {
            scattered_ray.direction = reflect(shading_ctx.wo, shading_ctx.normal);
            specularly_reflect_auxilary_rays(shading_ctx, scattered_ray, p_scattered_auxilary_rays);
            *specular_bounces_contribution *= specular_scattering.scattering_coeff;
        }
        else if (refract(shading_ctx.wo, shading_ctx.normal, eta, &scattered_ray.direction)) {
            ASSERT(specular_scattering.type == Specular_Scattering_Type::specular_transmission);
            specularly_transmit_auxilary_rays(shading_ctx, scattered_ray, eta, p_scattered_auxilary_rays);
            *specular_bounces_contribution *= (eta * eta) * specular_scattering.scattering_coeff;
        }
        else { // total internal reflection, nothing is transmitted
            *specular_bounces_contribution = Color_Black;
            return false;
        }
        if (!trace_ray(thread_ctx, scattered_ray, p_scattered_auxilary_rays))
            return false;
    }

    // check if we end up on specular surface after reaching max_specular_bounces
    if (specular_scattering.type != Specular_Scattering_Type::none) {
        *specular_bounces_contribution = Color_Black;
        return false;
    }
    return true;
}
