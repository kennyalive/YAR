#include "std.h"
#include "lib/common.h"
#include "path_tracing.h"

#include "bsdf.h"
#include "direct_lighting.h"
#include "scene_context.h"
#include "shading_context.h"
#include "thread_context.h"

ColorRGB trace_path(Thread_Context& thread_ctx, const Ray& ray, const Differential_Rays& differential_rays)
{
    const Scene_Context& scene_ctx = *thread_ctx.scene_context;
    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    const Raytracer_Config& rt_config = scene_ctx.raytracer_config;

    Path_Context& path_ctx = thread_ctx.path_context;
    Ray current_ray = ray;
    ColorRGB path_coeff = Color_White;
    bool ray_is_already_traced_by_delta_bounce = false;

    ColorRGB L;
    while (true) {
        if (!ray_is_already_traced_by_delta_bounce) {
            bool hit_found = trace_ray(thread_ctx, current_ray, path_ctx.bounce_count == 0 ? &differential_rays : nullptr);

            // Collect directly visible emitted light.
            if (path_ctx.bounce_count == 0) {
                if (hit_found)
                    L += path_coeff * get_emitted_radiance(thread_ctx);
                else if (scene_ctx.has_environment_light_sampler)
                    L += path_coeff * scene_ctx.environment_light_sampler.get_filtered_radiance_for_direction(shading_ctx.miss_ray.direction);

                if (rt_config.max_light_bounces == 0)
                    break;
            }

            // About area light check: in current design we don't have scattering on area light
            // sources (shading_ctx.bsdf == nullptr). Hitting area light ends path generation
            // (subsequent segments have no effect due to zero bsdf on area light).
            if (!hit_found || shading_ctx.area_light != Null_Light)
                break;
        }
        ray_is_already_traced_by_delta_bounce = false;

        float u_scattering_type = thread_ctx.pixel_sampler.get_next_1d_sample();
        float u_light_index = thread_ctx.pixel_sampler.get_next_1d_sample();
        float u_scattering_type_next_segment = thread_ctx.pixel_sampler.get_next_1d_sample();
        Vector2 u_light = thread_ctx.pixel_sampler.get_next_2d_sample();
        Vector2 u_bsdf = thread_ctx.pixel_sampler.get_next_2d_sample();
        Vector2 u_bsdf_next_segment = thread_ctx.pixel_sampler.get_next_2d_sample();

        thread_ctx.shading_context.initialize_scattering(thread_ctx, &u_scattering_type);

        if (!shading_ctx.delta_scattering_event) {
            ColorRGB direct_lighting = estimate_direct_lighting_from_single_sample(thread_ctx, u_light_index, u_light, u_bsdf, u_scattering_type);
            L += path_coeff * direct_lighting;

            path_ctx.bounce_count++;
            if (path_ctx.bounce_count == rt_config.max_light_bounces)
                break;

            Vector3 wi;
            float bsdf_pdf;
            ColorRGB f = shading_ctx.bsdf->sample(u_bsdf_next_segment, u_scattering_type_next_segment, shading_ctx.wo, &wi, &bsdf_pdf);
            if (f.is_black())
                break;

            path_coeff *= f * (std::abs(dot(shading_ctx.normal, wi)) / (bsdf_pdf * shading_ctx.bsdf_layer_selection_probability));

            current_ray.origin = shading_ctx.get_ray_origin_using_control_direction(wi);
            current_ray.direction = wi;
        }
        else {
            if (shading_ctx.bsdf) {
                ColorRGB direct_lighting = estimate_direct_lighting_from_single_sample(thread_ctx, u_light_index, u_light, u_bsdf, u_scattering_type);
                L += path_coeff * direct_lighting;
            }

            Delta_Scattering ds = shading_ctx.delta_scattering;

            Ray delta_ray;
            delta_ray.origin = shading_ctx.get_ray_origin_using_control_direction(ds.delta_direction);
            delta_ray.direction = ds.delta_direction;

            bool delta_hit_found = trace_ray(thread_ctx, delta_ray,
                ds.has_differential_rays ? &ds.differential_rays : nullptr);

            ColorRGB emitted_radiance;
            if (delta_hit_found)
                emitted_radiance = get_emitted_radiance(thread_ctx);
            else if (scene_ctx.has_environment_light_sampler)
                emitted_radiance = scene_ctx.environment_light_sampler.get_filtered_radiance_for_direction(
                    shading_ctx.miss_ray.direction);

            path_coeff *= ds.attenuation;
            L += path_coeff * emitted_radiance;

            path_ctx.bounce_count++;
            if (path_ctx.bounce_count == rt_config.max_light_bounces)
                break;

            path_ctx.perfect_specular_bounce_count++;

            if (!delta_hit_found || shading_ctx.area_light != Null_Light)
                break;

            ray_is_already_traced_by_delta_bounce = true;
        }

        // Apply russian roulette.
        if (path_ctx.bounce_count >= rt_config.russian_roulette_bounce_count_threshold) {

            // That's fine to get the next sample inside the above condition because that
            // condition is evaluated to the same value for all paths at the given depth.
            // The same reasoning explains why we can't move this call down even further
            // and to have it inside the next condition - that condition is a function
            // of the current path.
            float u_termination = thread_ctx.pixel_sampler.get_next_1d_sample();

            float max_coeff = std::max(path_coeff[0], std::max(path_coeff[1], path_coeff[2]));

            if (max_coeff < rt_config.russian_roulette_threshold) {
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
