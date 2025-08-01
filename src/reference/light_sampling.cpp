#include "std.h"
#include "lib/common.h"
#include "light_sampling.h"

#include "image_texture.h"
#include "intersection.h"
#include "shading_context.h"

#include "lib/light.h"

inline float transform_pdf_uv_to_solid_angle_measure(float pdf_uv, float sin_theta) {
    return pdf_uv / (2*Pi*Pi*sin_theta);
}

static Vector2 get_uv_from_direction(const Vector3& env_map_direction) {
    float phi = std::atan2(env_map_direction.y, env_map_direction.x);
    phi = phi < 0 ? phi + Pi2 : phi;
    float theta = std::acos(std::clamp(env_map_direction.z, -1.f, 1.f));

    Vector2 uv;
    uv[0] = std::clamp(phi * Pi2_Inv, 0.f, One_Minus_Epsilon);
    uv[1] = std::min(theta * Pi_Inv, One_Minus_Epsilon);
    return uv;
}

//
// Environment_Light_Sampler
//
ColorRGB Environment_Light_Sampler::sample(Vector2 u, Vector3* wi, float* pdf) const {
    float pdf_uv;
    Vector2 uv = radiance_distribution.sample(u, &pdf_uv);
    ASSERT(pdf_uv != 0.f);

    float phi = uv[0] * Pi2;
    float theta = uv[1] * Pi;

    float sin_theta = std::sin(theta);
    *pdf = transform_pdf_uv_to_solid_angle_measure(pdf_uv, sin_theta);

    *wi = { sin_theta * std::cos(phi), sin_theta * std::sin(phi), std::cos(theta) };
    *wi = transform_vector(light->light_to_world, *wi);

    // Do not filter environment map to ensure that sampled radiance values match pdf distribution map.
    // When doing filtering it's possible to get high variance (fireflies) when large radiance value
    // is smeared onto the low pdf region.
    ColorRGB Le = environment_map->sample_nearest(uv, 0, Wrap_Mode::clamp) * light->scale;
    return Le;
}

ColorRGB Environment_Light_Sampler::get_unfiltered_radiance_for_direction(const Vector3& world_direction) const {
    Vector3 env_map_direction = transform_vector(light->world_to_light, world_direction);
    Vector2 uv = get_uv_from_direction(env_map_direction);
    return environment_map->sample_nearest(uv, 0, Wrap_Mode::clamp) * light->scale;
}

ColorRGB Environment_Light_Sampler::get_filtered_radiance_for_direction(const Vector3& world_direction) const {
    Vector3 env_map_direction = transform_vector(light->world_to_light, world_direction);
    Vector2 uv = get_uv_from_direction(env_map_direction);
    return environment_map->sample_bilinear(uv, 0, Wrap_Mode::clamp) * light->scale;
}

float Environment_Light_Sampler::pdf(const Vector3& world_direction) const {
    Vector3 env_map_direction = transform_vector(light->world_to_light, world_direction);
    Vector2 uv = get_uv_from_direction(env_map_direction);
    float pdf_uv = radiance_distribution.pdf_uv(uv);

    float sin_theta = std::sqrt(std::max(0.f, 1.f -  env_map_direction.z * env_map_direction.z));

    float pdf = transform_pdf_uv_to_solid_angle_measure(pdf_uv, sin_theta);
    return pdf;
}

//
// Diffuse_Sphere_Light_Sampler
//
Diffuse_Sphere_Light_Sampler::Diffuse_Sphere_Light_Sampler(const Diffuse_Sphere_Light& light, const Vector3& shading_pos)
    : light(light)
    , shading_pos(shading_pos)
{
    axes[2] = shading_pos - light.position;
    d_center = axes[2].length();
    axes[2] /= d_center;
    coordinate_system_from_vector(axes[2], &axes[0], &axes[1]);

    float sin_theta_max = light.radius / d_center;
    cos_theta_max = std::sqrt(std::max(0.f, 1.f - sin_theta_max * sin_theta_max));

    cone_sampling_pdf = 1.f / (2.f * Pi * (1.f - cos_theta_max));
}

Vector3 Diffuse_Sphere_Light_Sampler::sample(Vector2 u) const {
    ASSERT(u < Vector2(1));

    float radius2 = light.radius * light.radius;
    float d_center2 = d_center * d_center;

    // theta and phi are computed based on uniform sampling of cone's solid angle
    float cos_theta = (1.f - u[0]) + u[0] * cos_theta_max;
    float phi = 2.f * Pi * u[1];

    // compute distance to the sample point, which is determined where (theta, phi) direction intersects the sphere
    float sin_theta2 = std::max(0.f, 1.f - cos_theta * cos_theta);
    float d_sample = d_center * cos_theta - std::sqrt(std::max(0.f, radius2 - d_center2 * sin_theta2));
    ASSERT(d_sample >= 0.f);

    // compute angle determined by the direction from the sphere center to the intersection point using the law of cosines
    float cos_alpha = (d_center2 + radius2 - d_sample * d_sample) / (2.f * d_center * light.radius);
    float sin_alpha = std::sqrt(std::max(0.f, 1.f - cos_alpha * cos_alpha));

    // compute direction from the sphere center to the sampled point
    Vector3 direction = (sin_alpha * std::cos(phi)) * axes[0] +  (sin_alpha * std::sin(phi)) * axes[1] + cos_alpha * axes[2];

    // finally we have a point on the sphere
    Vector3 p = light.radius * direction;
    p += light.position;
    return p;
}

bool Diffuse_Sphere_Light_Sampler::is_direction_inside_light_cone(const Vector3& wi) const {
    return dot(-axes[2], wi) >= cos_theta_max;
}

//
// Diffuse_Triangle_Mesh_Light_Sampler
//
Vector3 Diffuse_Triangle_Mesh_Light_Sampler::sample(Vector2 u, const Vector3& shading_pos, float* pdf) const
{
    float remapped_u0;
    float s = triangle_distribution.sample(u[0], nullptr, &remapped_u0);
    uint32_t triangle_index = std::min(uint32_t(s * mesh->get_triangle_count()), (uint32_t)mesh->get_triangle_count() - 1);
    u[0] = remapped_u0;

    Vector3 b = uniform_sample_triangle_baricentrics(u);

    Vector3 light_p = mesh->get_position(triangle_index, b);

    Vector3 light_n;
    if (!mesh->normals.empty()) {
        light_n = mesh->get_normal(triangle_index, b);
    }
    else {
        light_n = mesh->get_geometric_normal(triangle_index);
    }

    const Vector3 light_vec = light_p - shading_pos;
    float distance_to_light_sq = light_vec.length_squared();
    Vector3 wi = light_vec / std::sqrt(distance_to_light_sq);

    float light_n_dot_wi = dot(light_n, -wi);
    if (light_n_dot_wi <= 0.f) {
        // No emmission on the back side of the light. The caller will skip light contribution from this sample.
        // NOTE: for two-sided lights this check is not needed (currently only single sided lights are supported).
        *pdf = 0.f;
    }
    else {
        // Compute pdf with respect to solid angle measure.
        // This uses pdf with respect to area measure which is the inverse of the area value.
        *pdf = distance_to_light_sq / (light_n_dot_wi * mesh_area);
    }
    return light_p;
}

float Diffuse_Triangle_Mesh_Light_Sampler::pdf(const Vector3& shading_pos, const Vector3& wi, const Intersection& light_intersection) const
{
    const Triangle_Intersection& isect = light_intersection.triangle_intersection;
    ASSERT(mesh == isect.mesh);

    Vector3 light_n;
    if (!mesh->normals.empty()) {
        light_n = mesh->get_normal(isect.triangle_index, isect.barycentrics);
    }
    else {
        light_n = mesh->get_geometric_normal(isect.triangle_index);
    }

    float light_n_dot_wi = dot(light_n, -wi);
    if (light_n_dot_wi <= 0.f) {
        return 0.f;
    }
    else {
        float distance_to_light_sq = light_intersection.t * light_intersection.t;
        return distance_to_light_sq / (light_n_dot_wi * mesh_area);
    }
}
