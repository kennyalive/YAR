#include "std.h"
#include "lib/common.h"
#include "direct_lighting.h"

#include "bsdf.h"
#include "context.h"
#include "light_sampling.h"
#include "sampling.h"
#include "shading_context.h"

#include "lib/light.h"
#include "lib/math.h"

inline float mis_power_heuristic(float pdf1, float pdf2) {
    return pdf1*pdf1 / (pdf1*pdf1 + pdf2*pdf2);
}

static ColorRGB sample_lights(const Scene_Context& ctx, const Shading_Context& shading_ctx, pcg32_random_t* rng) {
    ColorRGB L;

    for (const Point_Light& light : ctx.lights.point_lights) {
        const Vector3 light_vec = (light.position - shading_ctx.P);
        const float light_dist = light_vec.length();
        const Vector3 light_dir = light_vec / light_dist;

        float n_dot_l = dot(shading_ctx.N, light_dir);
        if (n_dot_l <= 0.f)
            continue;

        Ray shadow_ray(shading_ctx.P, light_dir);
        bool in_shadow = ctx.acceleration_structure->intersect_any(shadow_ray, light_dist - 1e-4f);
        if (in_shadow)
            continue;

        ColorRGB bsdf = shading_ctx.bsdf->evaluate(shading_ctx.Wo, light_dir);
        L += bsdf * light.intensity * (n_dot_l / (light_dist * light_dist));
    }

    for (const Directional_Light& light : ctx.lights.directional_lights) {
        float n_dot_l = dot(shading_ctx.N, light.direction);
        if (n_dot_l <= 0.f)
            continue;

        Ray shadow_ray(shading_ctx.P, light.direction);
        bool in_shadow = ctx.acceleration_structure->intersect_any(shadow_ray, Infinity);
        if (in_shadow)
            continue;

        ColorRGB bsdf = shading_ctx.bsdf->evaluate(shading_ctx.Wo, light.direction);
        L += bsdf * light.irradiance * n_dot_l;
    }

    for (const Diffuse_Rectangular_Light& light : ctx.lights.diffuse_rectangular_lights) {
        ColorRGB L2;
        for (int i = 0; i < light.sample_count; i++) {
            Vector2 u{ 2.0f * random_float(rng) - 1.0f, 2.0f * random_float(rng) - 1.0f };
            Vector3 local_light_point = Vector3{ light.size.x / 2.0f * u.x, light.size.y / 2.0f * u.y, 0.0f };
            Vector3 light_point = transform_point(light.light_to_world_transform, local_light_point);

            const Vector3 light_vec = (light_point - shading_ctx.P);
            const float light_dist = light_vec.length();
            const Vector3 light_dir = light_vec / light_dist;

            Vector3 light_normal = light.light_to_world_transform.get_column(2);
            float light_n_dot_l = dot(light_normal, -light_dir);
            if (light_n_dot_l <= 0.f)
                continue;

            float n_dot_l = dot(shading_ctx.N, light_dir);
            if (n_dot_l <= 0.f)
                continue;

            Ray shadow_ray(shading_ctx.P, light_dir);
            bool in_shadow = ctx.acceleration_structure->intersect_any(shadow_ray, light_dist - 1e-3f);
            if (in_shadow)
                continue;

            ColorRGB bsdf = shading_ctx.bsdf->evaluate(shading_ctx.Wo, light_dir);
            L2 += (light.size.x * light.size.y) * light.emitted_radiance * bsdf * (n_dot_l * light_n_dot_l / (light_dist * light_dist));
        }
        L2 /= float(light.sample_count);
        L += L2;
    }

    for (auto [light_num, light] : enumerate(ctx.lights.diffuse_sphere_lights)) {
        Diffuse_Sphere_Light_Area_Sampler sampler(&ctx, rng, shading_ctx.P, (int)light_num);

        ColorRGB L2;
        for (int i = 0; i < light.sample_count; i++) {
            // Light sampling part of MIS.
            {
                Vector3 light_n = sampler.sample_direction_on_sphere();

                Vector3 p = light.radius * light_n;
                p = offset_ray_origin(p, light_n);
                p += light.position;

                Vector3 light_vector = p - shading_ctx.P;
                float distance = light_vector.length();
                Vector3 wi = light_vector / distance;

                float light_n_dot_wi = dot(light_n, -wi);
                if (light_n_dot_wi > 0.f) { // early light visiblity test
                    float n_dot_wi = dot(shading_ctx.N, wi);

                    bool scattering_possible = n_dot_wi > 0.f && shading_ctx.bsdf->reflection_scattering ||
                                               n_dot_wi < 0.f && shading_ctx.bsdf->transmission_scattering;

                    if (scattering_possible) {
                        ColorRGB f = shading_ctx.bsdf->evaluate(shading_ctx.Wo, wi);
                        if (!f.is_black()) {
                            Ray visibility_ray(shading_ctx.P, wi);
                            bool occluded = ctx.acceleration_structure->intersect_any(visibility_ray, distance);
                            if (!occluded) {
                                float bsdf_pdf = shading_ctx.bsdf->pdf(shading_ctx.Wo, wi);
                                float light_pdf = (sampler.area_pdf * distance * distance) / light_n_dot_wi; // convert to solid angle pdf
                                float mis_weight = mis_power_heuristic(light_pdf, bsdf_pdf);

                                L2 += (light.emitted_radiance * f) * (mis_weight * std::abs(n_dot_wi) / light_pdf);
                            }
                        }
                    }
                }
            }

            // BSDF sampling part of MIS.
            {
                Vector2 u{ random_float(rng), random_float(rng) };

                Vector3 wi;
                float bsdf_pdf;
                ColorRGB f = shading_ctx.bsdf->sample(u, shading_ctx.Wo, &wi, &bsdf_pdf);

                if (!f.is_black()) {
                    ASSERT(bsdf_pdf > 0.f);
                    float light_pdf = sampler.pdf(wi);
                    if (light_pdf != 0.f) {
                        ASSERT(light_pdf > 0.f);
                        float mis_weight = mis_power_heuristic(bsdf_pdf, light_pdf);
                        float n_dot_wi = dot(shading_ctx.N, wi);
                        L2 += (light.emitted_radiance * f) * (mis_weight * std::abs(n_dot_wi) / bsdf_pdf);
                    }
                }
            }
        }
        L2 /= float(light.sample_count);
        L += L2;
    }

    if (ctx.has_environment_light_sampler) {
        ColorRGB L2;
        for (int i = 0; i < ctx.environment_light_sampler.light->sample_count; i++) {
            // Light sampling part of MIS.
            {
                Vector2 u{ random_float(rng), random_float(rng) };
                Light_Sample light_sample = ctx.environment_light_sampler.sample(u);

                float N_dot_Wi = dot(shading_ctx.N, light_sample.Wi);

                bool scattering_possible = N_dot_Wi > 0.f && shading_ctx.bsdf->reflection_scattering ||
                                           N_dot_Wi < 0.f && shading_ctx.bsdf->transmission_scattering;

                if (scattering_possible) {
                    ColorRGB f = shading_ctx.bsdf->evaluate(shading_ctx.Wo, light_sample.Wi);
                    if (!f.is_black()) {
                        Ray shadow_ray(shading_ctx.P, light_sample.Wi);
                        bool occluded = ctx.acceleration_structure->intersect_any(shadow_ray, Infinity);
                        if (!occluded) {
                            float bsdf_pdf = shading_ctx.bsdf->pdf(shading_ctx.Wo, light_sample.Wi);
                            float mis_weight = mis_power_heuristic(light_sample.pdf, bsdf_pdf);
                            L2 += (light_sample.Le * f) * (mis_weight * std::abs(N_dot_Wi) / light_sample.pdf);
                        }
                    }
                }
            }

            // BSDF sampling part of MIS.
            {
                Vector3 wi;
                float bsdf_pdf;

                Vector2 u{ random_float(rng), random_float(rng) };
                ColorRGB f = shading_ctx.bsdf->sample(u, shading_ctx.Wo, &wi, &bsdf_pdf);

                if (!f.is_black()) {
                    ASSERT(bsdf_pdf > 0.f);
                    ColorRGB Le = ctx.environment_light_sampler.get_radiance_for_direction(wi);
                    if (!Le.is_black()) {
                        Ray shadow_ray(shading_ctx.P, wi);
                        bool occluded = ctx.acceleration_structure->intersect_any(shadow_ray, Infinity);
                        if (!occluded) {
                            float light_pdf = ctx.environment_light_sampler.pdf(wi);
                            float mis_weight = mis_power_heuristic(bsdf_pdf, light_pdf);
                            float N_dot_Wi = dot(shading_ctx.N, wi);
                            L2 += (Le * f) * (mis_weight * std::abs(N_dot_Wi) / bsdf_pdf);
                        }
                    }
                }
            }
        }
        L2 /= float(ctx.environment_light_sampler.light->sample_count);
        L += L2;
    }

    return L;
}

static ColorRGB reflect_from_mirror_surface(const Scene_Context& ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, pcg32_random_t* rng, int max_specular_depth) {
    Ray reflected_ray;
    reflected_ray.origin = shading_ctx.P;
    reflected_ray.direction = shading_ctx.N * (2.f * dot(shading_ctx.N, shading_ctx.Wo)) - shading_ctx.Wo;

    Intersection isect;
    if (ctx.acceleration_structure->intersect(reflected_ray, isect)) {
        Shading_Point_Rays rays;
        rays.incident_ray = reflected_ray;
        // TODO: generate auxilary rays for mirror?
        rays.auxilary_ray_dx_offset = reflected_ray;
        rays.auxilary_ray_dy_offset = reflected_ray;

        Shading_Context shading_ctx2(ctx, thread_ctx, rays, isect);
        ColorRGB reflected_radiance = estimate_direct_lighting(ctx, thread_ctx, shading_ctx2, rng, max_specular_depth);
        return reflected_radiance * shading_ctx.mirror_reflectance;
    }
    else if (ctx.has_environment_light_sampler) {
        return ctx.environment_light_sampler.get_radiance_for_direction(reflected_ray.direction);
    }
    else {
        return Color_Black;
    }
}

ColorRGB estimate_direct_lighting(const Scene_Context& ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, pcg32_random_t* rng, int max_specular_depth)
{
    ColorRGB L;
    if (shading_ctx.mirror_surface) {
        if (max_specular_depth > 0) {
            L = reflect_from_mirror_surface(ctx, thread_ctx, shading_ctx, rng, --max_specular_depth);
        }
    }
    else if (shading_ctx.area_light != Null_Light) {
        if (shading_ctx.area_light.type == Light_Type::diffuse_rectangular) {
            L = ctx.lights.diffuse_rectangular_lights[shading_ctx.area_light.index].emitted_radiance;
        }
        else {
            ASSERT(shading_ctx.area_light.type == Light_Type::diffuse_sphere);
            L = ctx.lights.diffuse_sphere_lights[shading_ctx.area_light.index].emitted_radiance;
        }
    }
    else {
        L = sample_lights(ctx, shading_ctx, rng);
    }
    return L;
}
