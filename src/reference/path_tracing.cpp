#include "std.h"
#include "lib/common.h"
#include "path_tracing.h"

#include "bsdf.h"
#include "context.h"
#include "direct_lighting.h"
#include "shading_context.h"

constexpr int bounce_count_when_to_apply_russian_roulette = 3;

ColorRGB estimate_path_contribution(const Scene_Context& scene_ctx, Thread_Context& thread_ctx, const Ray& ray, const Auxilary_Rays& auxilary_rays) {
    const Shading_Context& shading_ctx = thread_ctx.shading_context;
    const int max_bounces = scene_ctx.scene->raytracer_config.max_light_bounces;
    ASSERT(max_bounces >= 1); // at least direct lighting component

    int bounce_count = 0;
    ColorRGB path_coeff = Color_White;
    Ray current_ray = ray;
    Auxilary_Rays current_auxilary_rays = auxilary_rays;

    ColorRGB L;
    while (true) {
        ColorRGB specular_attenuation;
        if (!trace_ray(scene_ctx, thread_ctx, &current_ray, bounce_count == 0 ? &current_auxilary_rays : nullptr, &specular_attenuation, 10)) {
            if (bounce_count > 0) {
                break;
            }
            if (scene_ctx.has_environment_light_sampler) {
                return specular_attenuation * scene_ctx.environment_light_sampler.get_radiance_for_direction(ray.direction);
            }
            return Color_Black;
        }

        if (bounce_count == 0 && shading_ctx.area_light != Null_Light) {
            if (shading_ctx.area_light.type == Light_Type::diffuse_rectangular) {
                return specular_attenuation * scene_ctx.lights.diffuse_rectangular_lights[shading_ctx.area_light.index].emitted_radiance;
            }
            else {
                ASSERT(shading_ctx.area_light.type == Light_Type::diffuse_sphere);
                return specular_attenuation * scene_ctx.lights.diffuse_sphere_lights[shading_ctx.area_light.index].emitted_radiance;
            }
        }
        else if (shading_ctx.area_light != Null_Light) {
            // In current design we don't have scattering on area light sources (shading_ctx.bsdf == nullptr).
            // It means if we hit an area light (except the first bounce) then the path contribution will be zero (assume bsdf == 0 on area light).
            break;
        }

        path_coeff *= specular_attenuation;

        float u_light_index = thread_ctx.pixel_sampler.get_next_1d_sample();
        Vector2 u_light = thread_ctx.pixel_sampler.get_next_2d_sample();
        Vector2 u_bsdf = thread_ctx.pixel_sampler.get_next_2d_sample();

        // Add contribution of the current path.
        L += path_coeff * estimate_direct_lighting_from_single_sample(scene_ctx, shading_ctx, u_light_index, u_light, u_bsdf);

        bounce_count++;
        if (bounce_count == max_bounces)
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
        ColorRGB f = shading_ctx.bsdf->sample(u, shading_ctx.Wo, &wi, &bsdf_pdf);
        if (f.is_black())
            break;

        path_coeff *= f * (std::abs(dot(shading_ctx.N, wi)) / bsdf_pdf);
        current_ray = Ray(shading_ctx.P, wi);
    }
    return L;
}
