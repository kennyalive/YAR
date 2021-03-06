#include "std.h"
#include "lib/common.h"
#include "path_tracing.h"

#include "bsdf.h"
#include "context.h"
#include "direct_lighting.h"
#include "shading_context.h"

constexpr int bounce_count_when_to_apply_russian_roulette = 3;

ColorRGB estimate_path_contribution(Thread_Context& thread_ctx, const Ray& ray, const Auxilary_Rays& auxilary_rays) {
    const Scene_Context& scene_ctx = *thread_ctx.scene_context;
    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    Path_Context& path_ctx = thread_ctx.path_context;

    const int max_bounces = scene_ctx.scene->raytracer_config.max_light_bounces;

    ColorRGB path_coeff = Color_White;
    Ray current_ray = ray;

    ColorRGB L;
    while (true) {
        const Auxilary_Rays* p_auxilary_rays = (path_ctx.bounce_count == 0) ? &auxilary_rays : nullptr;
        bool hit_found = trace_ray(thread_ctx, current_ray, p_auxilary_rays);

        // If we hit perfect specular surface then keep bouncing until we reach finite bsdf or exit the scene.
        if (shading_ctx.specular_scattering.type != Specular_Scattering_Type::none) {
            ColorRGB specular_attenuation = Color_White;
            hit_found = trace_specular_bounces(thread_ctx, max_bounces, &specular_attenuation);
            path_coeff *= specular_attenuation;

            if (hit_found)
                L += path_coeff * get_emitted_radiance(thread_ctx);
            else if (scene_ctx.has_environment_light_sampler)
                L += path_coeff * scene_ctx.environment_light_sampler.get_radiance_for_direction(shading_ctx.miss_ray.direction);

            // check if we reached max bounce limit due to bounces on specular surfaces
            if (path_ctx.bounce_count == max_bounces)
                break;
        }

        // Collect directly visible emitted light.
        if (path_ctx.bounce_count == 0) {
            if (hit_found)
                L += get_emitted_radiance(thread_ctx);
            else if (scene_ctx.has_environment_light_sampler)
                L += scene_ctx.environment_light_sampler.get_radiance_for_direction(shading_ctx.miss_ray.direction);
        }

        // About area light check: in current design we don't have scattering on area light
        // sources (shading_ctx.bsdf == nullptr). Hitting area light ends path generation
        // (subsequent segments have no effect due to zero bsdf on area light).
        if (!hit_found || shading_ctx.area_light != Null_Light)
            break;

        float u_light_index = thread_ctx.pixel_sampler.get_next_1d_sample();
        Vector2 u_light = thread_ctx.pixel_sampler.get_next_2d_sample();
        Vector2 u_bsdf = thread_ctx.pixel_sampler.get_next_2d_sample();

        // Add contribution of the current path.
        L += path_coeff * estimate_direct_lighting_from_single_sample(thread_ctx, u_light_index, u_light, u_bsdf);

        if (++path_ctx.bounce_count == max_bounces)
            break;

        // Apply russian roulette.
        if (path_ctx.bounce_count >= bounce_count_when_to_apply_russian_roulette) {
            // That's fine to get the next sample inside condition because condition
            // is evaluated to the same value for all threads.
            float u_termination = thread_ctx.pixel_sampler.get_next_1d_sample();

            float max_coeff = std::max(path_coeff[0], std::max(path_coeff[1], path_coeff[2]));
            float termination_probability = std::max(0.05f, 1.f - max_coeff);

            if (u_termination < termination_probability)
                break;

            path_coeff /= 1 - termination_probability;
        }

        // Generate next path segment.
        Vector3 wi;
        float bsdf_pdf;

        Vector2 u = thread_ctx.pixel_sampler.get_next_2d_sample();
        ColorRGB f = shading_ctx.bsdf->sample(u, shading_ctx.wo, &wi, &bsdf_pdf);
        if (f.is_black())
            break;

        path_coeff *= f * (std::abs(dot(shading_ctx.normal, wi)) / bsdf_pdf);
        current_ray = Ray{shading_ctx.position, wi};
    }
    return L;
}
