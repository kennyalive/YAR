#include "lib/common.h"
#include "lib/random.h"
#include "light.h"
#include "material.h"

Diffuse_Rectangular_Light::Diffuse_Rectangular_Light(const RGB_Diffuse_Rectangular_Light_Data& light_data) {
    light_to_world_transform = light_data.light_to_world_transform;
    emitted_radiance = light_data.emitted_radiance;
    size = light_data.size;
    area = size.x * size.y;
    shadow_ray_count = light_data.shadow_ray_count;
}

ColorRGB compute_direct_lighting(
    const Local_Geometry& local_geom,
    const TwoLevel_KdTree* acceleration_structure,
    const Lights& lights,
    const Vector3& wo,
    Material_Handle material)
{
    ColorRGB L{};
    for (const Point_Light& light : lights.point_lights) {
        const Vector3 light_vec = (light.position - local_geom.position);
        const float light_dist = light_vec.length();
        const Vector3 light_dir = light_vec / light_dist;

        float n_dot_l = dot(local_geom.normal, light_dir);
        if (n_dot_l <= 0.f)
            continue;

        Ray shadow_ray(local_geom.position + local_geom.normal * 1e-3f, light_dir);
        float any_intersection_dist = acceleration_structure->intersect_any(shadow_ray);
        bool in_shadow = any_intersection_dist + 1e-4f < light_dist;
        if (in_shadow)
            continue;

        ColorRGB bsdf = compute_bsdf(material, light_dir, wo);
        L += bsdf * light.intensity * (n_dot_l / (light_dist * light_dist));
    }

    static uint64_t seed = 0;
    pcg32_random_t rng;
    pcg32_srandom_r(&rng, 0, seed++);

    for (const Diffuse_Rectangular_Light& light : lights.diffuse_rectangular_lights) {
        for (int i = 0; i < light.shadow_ray_count; i++) {
            Vector2 u{2.0f * random_float(&rng) - 1.0f, 2.0f * random_float(&rng) - 1.0f};
            Vector3 local_light_point = Vector3{light.size.x/2.0f * u.x, light.size.y/2.0f * u.y, 0.0f};

            Vector3 light_point = transform_point(light.light_to_world_transform, local_light_point);

            const Vector3 light_vec = (light_point - local_geom.position);
            const float light_dist = light_vec.length();
            const Vector3 light_dir = light_vec / light_dist;

            Vector3 light_normal = light.light_to_world_transform.get_column(2);
            float light_n_dot_l = dot(light_normal, -light_dir);
            if (light_n_dot_l <= 0.f)
                continue;

            float n_dot_l = dot(local_geom.normal, light_dir);
            if (n_dot_l <= 0.f)
                continue;

            Ray shadow_ray(local_geom.position + local_geom.normal * 1e-3f, light_dir);
            float any_intersection_dist = acceleration_structure->intersect_any(shadow_ray);
            bool in_shadow = any_intersection_dist + 1e-4f < light_dist;
            if (in_shadow)
                continue;

            ColorRGB bsdf = compute_bsdf(material, light_dir, wo);
            L += light.area * light.emitted_radiance * bsdf * (n_dot_l * light_n_dot_l / (light_dist * light_dist));
        }
        L /= float(light.shadow_ray_count);
    }
    return L;
}
