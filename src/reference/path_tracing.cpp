#include "std.h"
#include "lib/common.h"
#include "path_tracing.h"

#include "bsdf.h"
#include "context.h"
#include "direct_lighting.h"
#include "shading_context.h"

#include "lib/scene.h"

ColorRGB estimate_path_contribution(Thread_Context& thread_ctx, const Ray& ray,
    const Differential_Rays& differential_rays)
{
    const Scene_Context& scene_ctx = *thread_ctx.scene_context;
    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    Path_Context& path_ctx = thread_ctx.path_context;
    const Raytracer_Config& rt_config = scene_ctx.raytracer_config;

    const int max_bounces = rt_config.max_light_bounces;
    const int max_differential_ray_bounces = rt_config.max_differential_ray_specular_bounces;

    Ray current_ray = ray;
    ColorRGB path_coeff = Color_White;

    // These differential rays are computed after scattering on delta surfaces.
    Differential_Rays specular_differential_rays;
    bool use_specular_differential_rays = false;

    ColorRGB L;
    while (true) {
        const bool sample_emitted_radiance_after_delta_layer =
            shading_ctx.specular_scattering.sample_delta_direction ||
            shading_ctx.specular_scattering.type != Specular_Scattering_Type::none;

        const Differential_Rays* p_differential_rays = nullptr;
        if (path_ctx.bounce_count == 0) {
            p_differential_rays = &differential_rays;
        }
        else if (use_specular_differential_rays) {
            p_differential_rays = &specular_differential_rays;
            use_specular_differential_rays = false;
        }

        bool hit_found = trace_ray(thread_ctx, current_ray, p_differential_rays);

        // Collect directly visible and delta scattered emitted light.
        if (path_ctx.bounce_count == 0 || sample_emitted_radiance_after_delta_layer) {
            if (hit_found)
                L += path_coeff * get_emitted_radiance(thread_ctx);
            else if (scene_ctx.has_environment_light_sampler)
                L += path_coeff * scene_ctx.environment_light_sampler.get_filtered_radiance_for_direction(shading_ctx.miss_ray.direction);

            if (path_ctx.bounce_count == max_bounces)
                break;
        }

        // About area light check: in current design we don't have scattering on area light
        // sources (shading_ctx.bsdf == nullptr). Hitting area light ends path generation
        // (subsequent segments have no effect due to zero bsdf on area light).
        if (!hit_found || shading_ctx.area_light != Null_Light)
            break;

        float u_light_index = thread_ctx.pixel_sampler.get_next_1d_sample();
        Vector2 u_light_mis = thread_ctx.pixel_sampler.get_next_2d_sample();
        Vector2 u_bsdf_mis = thread_ctx.pixel_sampler.get_next_2d_sample();
        Vector2 u_bsdf_next_segment = thread_ctx.pixel_sampler.get_next_2d_sample();

        if (shading_ctx.specular_scattering.type == Specular_Scattering_Type::none) {
            // Sample light and add contribution from the current path.
            ColorRGB direct_lighting = estimate_direct_lighting_from_single_sample(thread_ctx, u_light_index, u_light_mis, u_bsdf_mis);
            L += path_coeff * direct_lighting;

            if (++path_ctx.bounce_count == max_bounces)
                break;

            // Generate next path segment.
            Vector3 wi;
            if (shading_ctx.specular_scattering.sample_delta_direction) {
                wi = shading_ctx.specular_scattering.delta_direction;
                path_coeff *= shading_ctx.specular_scattering.scattering_coeff;
            }
            else {
                float bsdf_pdf;
                ColorRGB f = shading_ctx.bsdf->sample(u_bsdf_next_segment, shading_ctx.wo, &wi, &bsdf_pdf);
                if (f.is_black())
                    break;
                path_coeff *= f * (shading_ctx.specular_scattering.finite_scattering_weight * std::abs(dot(shading_ctx.normal, wi)) / bsdf_pdf);
            }
            current_ray.origin = shading_ctx.get_ray_origin_using_control_direction(wi);
            current_ray.direction = wi;
        }
        else if (shading_ctx.specular_scattering.type == Specular_Scattering_Type::specular_reflection) {
            current_ray.direction = reflect(shading_ctx.wo, shading_ctx.normal);
            current_ray.origin = shading_ctx.get_ray_origin_using_control_direction(current_ray.direction);
            if (shading_ctx.has_dxdy_derivatives && path_ctx.bounce_count < max_differential_ray_bounces) {
                specular_differential_rays = shading_ctx.compute_differential_rays_for_specular_reflection(current_ray);
                use_specular_differential_rays = true;
            }
            path_coeff *= shading_ctx.specular_scattering.scattering_coeff;
            path_ctx.perfect_specular_bounce_count++;
            path_ctx.bounce_count++;
        }
        else if (shading_ctx.specular_scattering.type == Specular_Scattering_Type::specular_transmission) {
            const float eta = shading_ctx.specular_scattering.etaI_over_etaT;
            const bool refracted = refract(shading_ctx.wo, shading_ctx.normal, eta, &current_ray.direction);
            ASSERT(refracted); // specular_transmission event should never be selected when total internal reflection happens
            current_ray.origin = shading_ctx.get_ray_origin_using_control_direction(current_ray.direction);
            if (shading_ctx.has_dxdy_derivatives && path_ctx.bounce_count < max_differential_ray_bounces) {
                specular_differential_rays = shading_ctx.compute_differential_rays_for_specular_transmission(current_ray, eta);
                use_specular_differential_rays = true;
            }
            path_coeff *= shading_ctx.specular_scattering.scattering_coeff;
            path_ctx.perfect_specular_bounce_count++;
            path_ctx.bounce_count++;
        }

        // Apply russian roulette.
        if (path_ctx.bounce_count >= rt_config.russian_roulette_bounce_count_threshold) {

            // That's fine to get the next sample inside the above condition because that
            // condition is evaluated to the same value for all paths at the given depth.
            // The same reasoning explains why we can't move this call down even further
            // and to have it inside the next condition - that condition is a function
            // of the current path and the condition evaluation result is not the same among
            // different paths at the given depth.
            float u_termination = thread_ctx.pixel_sampler.get_next_1d_sample();

            float max_coeff = std::max(path_coeff[0], std::max(path_coeff[1], path_coeff[2]));

            if (max_coeff < rt_config.russian_roulette_radiance_threshold) {
                float termination_probability = std::max(0.05f, 1.f - max_coeff);
                if (u_termination < termination_probability) {
                    break;
                }
                path_coeff /= 1.f - termination_probability;
            }
        }
    }
    return L;
}
