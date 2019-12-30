#include "std.h"
#include "lib/common.h"
#include "direct_lighting.h"

#include "ray_lib.h"
#include "render_context.h"
#include "scattering.h"
#include "shading_context.h"

#include "lib/light.h"

ColorRGB compute_direct_lighting(const Render_Context& ctx, const Shading_Context& shading_ctx, pcg32_random_t* rng)
{
    ColorRGB L;
    for (const Point_Light& light : ctx.lights.point_lights) {
        Vector3 surface_point = offset_ray_origin(shading_ctx.P, shading_ctx.Ng);

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

    for (const Diffuse_Rectangular_Light& light : ctx.lights.diffuse_rectangular_lights) {
        for (int i = 0; i < light.shadow_ray_count; i++) {
            Vector2 u{2.0f * random_float(rng) - 1.0f, 2.0f * random_float(rng) - 1.0f};
            Vector3 local_light_point = Vector3{light.size.x/2.0f * u.x, light.size.y/2.0f * u.y, 0.0f};
            Vector3 light_point = transform_point(light.light_to_world_transform, local_light_point);

            Vector3 surface_point = offset_ray_origin(shading_ctx.P, shading_ctx.Ng);

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
            L += (light.size.x * light.size.y) * light.emitted_radiance * bsdf * (n_dot_l * light_n_dot_l / (light_dist * light_dist));
        }
        L /= float(light.shadow_ray_count);
    }

    if (shading_ctx.area_light != Null_Light) {
        ASSERT(shading_ctx.area_light.type == Light_Type::diffuse_rectangular);
        L += ctx.lights.diffuse_rectangular_lights[shading_ctx.area_light.index].emitted_radiance;
    }

    return L;
}
