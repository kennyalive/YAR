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
        Vector3 light_vec = (light.position - local_geom.position);
        float light_dist_sq_inv = 1.f / light_vec.length_squared();
        Vector3 light_dir = light_vec * std::sqrt(light_dist_sq_inv);

        ColorRGB bsdf = compute_bsdf(material, light_dir, wo);

        L += bsdf * light.intensity * (light_dist_sq_inv * std::max(0.f, dot(local_geom.normal, light_dir)));
    }
    return L;
}
