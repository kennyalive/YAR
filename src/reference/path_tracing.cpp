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
    const int max_bounces = scene_ctx.scene->raytracer_config.max_light_bounces;

    int bounce_count = 0;
    ColorRGB path_coeff = Color_White;
    Ray current_ray = ray;

    ColorRGB L;
    while (true) {
        ColorRGB specular_bounces_contribution = Color_White;
        const Auxilary_Rays* current_auxilary_rays = (bounce_count == 0) ? &auxilary_rays : nullptr;

        bool is_specular_bounce = false;
        bool hit_found = trace_ray(thread_ctx, current_ray, current_auxilary_rays);
        if (hit_found && shading_ctx.specular_scattering.type != Specular_Scattering_Type::none) {
            hit_found = trace_specular_bounces(thread_ctx, current_auxilary_rays, max_bounces, &bounce_count, &specular_bounces_contribution);
            is_specular_bounce = true;
        }

        if (!hit_found) {
            if ((bounce_count == 0 || is_specular_bounce) && scene_ctx.has_environment_light_sampler) {
                L += specular_bounces_contribution * scene_ctx.environment_light_sampler.get_radiance_for_direction(shading_ctx.miss_ray.direction);
            }
            break;
        }

        if (shading_ctx.area_light != Null_Light) {
            if (bounce_count == 0 || is_specular_bounce) {
                L += specular_bounces_contribution * get_emitted_radiance(thread_ctx);
            }
            // In current design we don't have scattering on area light sources (shading_ctx.bsdf == nullptr).
            // Hitting area light ends path generation (subsequent segments have no effect due to zero bsdf on area light).
            break;
        }

        // check if we reached max bounce limit due to bounces on specular surfaces
        if (bounce_count == max_bounces)
            break;

        path_coeff *= specular_bounces_contribution;

        float u_light_index = thread_ctx.pixel_sampler.get_next_1d_sample();
        Vector2 u_light = thread_ctx.pixel_sampler.get_next_2d_sample();
        Vector2 u_bsdf = thread_ctx.pixel_sampler.get_next_2d_sample();

        // Add contribution of the current path.
        L += path_coeff * estimate_direct_lighting_from_single_sample(thread_ctx, u_light_index, u_light, u_bsdf);

        if (++bounce_count == max_bounces)
            break;

        // Apply russian roulette.
        if (bounce_count >= bounce_count_when_to_apply_russian_roulette) {
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
