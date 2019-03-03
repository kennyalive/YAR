#include "lib/common.h"
#include "light.h"
#include "material.h"

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
    return L;
}
