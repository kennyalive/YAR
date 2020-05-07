#include "std.h"
#include "lib/common.h"
#include "direct_lighting.h"

#include "bsdf.h"
#include "context.h"
#include "ray_lib.h"
#include "shading_context.h"

#include "lib/light.h"

static ColorRGB sample_lights(const Render_Context& ctx, const Shading_Context& shading_ctx, pcg32_random_t* rng) {
    ColorRGB L;
    Vector3 surface_point = offset_ray_origin(shading_ctx.P, shading_ctx.Ng);

    for (const Point_Light& light : ctx.lights.point_lights) {
        const Vector3 light_vec = (light.position - surface_point);
        const float light_dist = light_vec.length();
        const Vector3 light_dir = light_vec / light_dist;

        float n_dot_l = dot(shading_ctx.N, light_dir);
        if (n_dot_l <= 0.f)
            continue;

        Ray shadow_ray(surface_point, light_dir);
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

        Ray shadow_ray(surface_point, light.direction);
        bool in_shadow = ctx.acceleration_structure->intersect_any(shadow_ray, Infinity);
        if (in_shadow)
            continue;

        ColorRGB bsdf = shading_ctx.bsdf->evaluate(shading_ctx.Wo, light.direction);
        L += bsdf * light.irradiance * n_dot_l;
    }

    for (const Diffuse_Rectangular_Light& light : ctx.lights.diffuse_rectangular_lights) {
        ColorRGB L2;
        for (int i = 0; i < light.shadow_ray_count; i++) {
            Vector2 u{ 2.0f * random_float(rng) - 1.0f, 2.0f * random_float(rng) - 1.0f };
            Vector3 local_light_point = Vector3{ light.size.x / 2.0f * u.x, light.size.y / 2.0f * u.y, 0.0f };
            Vector3 light_point = transform_point(light.light_to_world_transform, local_light_point);

            const Vector3 light_vec = (light_point - surface_point);
            const float light_dist = light_vec.length();
            const Vector3 light_dir = light_vec / light_dist;

            Vector3 light_normal = light.light_to_world_transform.get_column(2);
            float light_n_dot_l = dot(light_normal, -light_dir);
            if (light_n_dot_l <= 0.f)
                continue;

            float n_dot_l = dot(shading_ctx.N, light_dir);
            if (n_dot_l <= 0.f)
                continue;

            Ray shadow_ray(surface_point, light_dir);
            bool in_shadow = ctx.acceleration_structure->intersect_any(shadow_ray, light_dist - 1e-3f);
            if (in_shadow)
                continue;

            ColorRGB bsdf = shading_ctx.bsdf->evaluate(shading_ctx.Wo, light_dir);
            L2 += (light.size.x * light.size.y) * light.emitted_radiance * bsdf * (n_dot_l * light_n_dot_l / (light_dist * light_dist));
        }
        L2 /= float(light.shadow_ray_count);
        L += L2;
    }
    return L;
}

static ColorRGB reflect_from_mirror_surface(const Render_Context& ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, pcg32_random_t* rng, int max_specular_depth) {
    Ray reflected_ray;
    reflected_ray.origin = offset_ray_origin(shading_ctx.P, shading_ctx.Ng);
    reflected_ray.direction = shading_ctx.N * (2.f * dot(shading_ctx.N, shading_ctx.Wo)) - shading_ctx.Wo;

    Intersection isect;
    if (!ctx.acceleration_structure->intersect(reflected_ray, isect))
        return ColorRGB{};

    Shading_Point_Rays rays;
    rays.incident_ray = reflected_ray;
    // TODO: generate auxilary rays for mirror?
    rays.auxilary_ray_dx_offset = reflected_ray;
    rays.auxilary_ray_dy_offset = reflected_ray;

    Shading_Context shading_ctx2(ctx, thread_ctx, rays, isect);
    ColorRGB reflected_radiance = estimate_direct_lighting(ctx, thread_ctx, shading_ctx2, rng, max_specular_depth);
    return reflected_radiance * shading_ctx.mirror_reflectance;
}

ColorRGB estimate_direct_lighting(const Render_Context& ctx, Thread_Context& thread_ctx, const Shading_Context& shading_ctx, pcg32_random_t* rng, int max_specular_depth)
{
    ColorRGB L;
    if (shading_ctx.mirror_surface) {
        if (max_specular_depth > 0) {
            L = reflect_from_mirror_surface(ctx, thread_ctx, shading_ctx, rng, --max_specular_depth);
        }
    }
    else {
        L = sample_lights(ctx, shading_ctx, rng);
    }
    
    if (shading_ctx.area_light != Null_Light) {
        ASSERT(shading_ctx.area_light.type == Light_Type::diffuse_rectangular);
        L += ctx.lights.diffuse_rectangular_lights[shading_ctx.area_light.index].emitted_radiance;
    }
    return L;
}
