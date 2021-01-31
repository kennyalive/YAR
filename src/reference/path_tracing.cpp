#include "std.h"
#include "lib/common.h"
#include "path_tracing.h"

#include "bsdf.h"
#include "context.h"
#include "direct_lighting.h"
#include "shading_context.h"

constexpr int path_length_to_apply_russian_roulette_ = 4;

ColorRGB estimate_path_contribution(const Scene_Context& scene_ctx, Thread_Context& thread_ctx, const Footprint_Tracking_Ray& ray) {
    Intersection isect;
    if (!scene_ctx.acceleration_structure->intersect(ray.main_ray, isect)) {
        if (scene_ctx.has_environment_light_sampler)
            return scene_ctx.environment_light_sampler.get_radiance_for_direction(ray.main_ray.direction);
        else
            return Color_Black;
    }

    Shading_Context& shading_ctx = thread_ctx.shading_context;
     shading_ctx.initialize_from_intersection(scene_ctx, thread_ctx, ray, isect);

    // debug visualization of samples with adjusted shading normal.
    /*if (shading_ctx.shading_normal_adjusted) {
        tile.add_sample(film_pos, Color_Red); 
        continue;
    }*/

    ASSERT(!shading_ctx.mirror_surface);
    const int max_path_length = scene_ctx.scene->raytracer_config.max_path_length;

    ColorRGB L;
    if (shading_ctx.area_light != Null_Light) {
        if (shading_ctx.area_light.type == Light_Type::diffuse_rectangular) {
            L = scene_ctx.lights.diffuse_rectangular_lights[shading_ctx.area_light.index].emitted_radiance;
        }
        else {
            ASSERT(shading_ctx.area_light.type == Light_Type::diffuse_sphere);
            L = scene_ctx.lights.diffuse_sphere_lights[shading_ctx.area_light.index].emitted_radiance;
        }
    }
    else {
        int path_length = 1; // start with the camera ray
        ColorRGB path_coeff(1.f);

        while (path_length != max_path_length) {
            path_length++;

            if (path_length >= path_length_to_apply_russian_roulette_) {
                // That's fine to get the next sample inside condition because condition
                // is evaluated to the same value for all threads.
                float u_termination = thread_ctx.pixel_sampler.get_next_1d_sample();

                float max_coeff = std::max(path_coeff[0], std::max(path_coeff[1], path_coeff[2]));
                float termination_probability = std::max(0.05f, 1.f - max_coeff);

                if (u_termination < termination_probability)
                    break;

                path_coeff /= 1 - termination_probability;
            }

            // Evaluate light contribution for current path.
            float u_light_index = thread_ctx.pixel_sampler.get_next_1d_sample();
            Vector2 u_light = thread_ctx.pixel_sampler.get_next_2d_sample();
            Vector2 u_bsdf = thread_ctx.pixel_sampler.get_next_2d_sample();
            L += path_coeff * estimate_direct_lighting_from_single_sample(scene_ctx, shading_ctx, u_light_index, u_light, u_bsdf);

            // Generate next path segment.
            if (path_length != max_path_length) {
                Vector2 u = thread_ctx.pixel_sampler.get_next_2d_sample();

                Vector3 wi;
                float bsdf_pdf;
                ColorRGB f = shading_ctx.bsdf->sample(u, shading_ctx.Wo, &wi, &bsdf_pdf);
                if (f.is_black())
                    break;

                path_coeff *= f * (std::abs(dot(shading_ctx.N, wi)) / bsdf_pdf);

                Ray next_segment_ray(shading_ctx.P, wi);

                Intersection isect;
                if (!scene_ctx.acceleration_structure->intersect(next_segment_ray, isect))
                    break;

                if (isect.scene_object->area_light != Null_Light)
                    break;

                Footprint_Tracking_Ray ray;
                ray.main_ray = next_segment_ray;
                ray.auxilary_ray_dx_offset = next_segment_ray;
                ray.auxilary_ray_dy_offset = next_segment_ray;

                shading_ctx.initialize_from_intersection(scene_ctx, thread_ctx, ray, isect);
            }
        }
    }
    return L;
}
