#include "std.h"
#include "lib/common.h"
#include "light_sampling.h"

#include "image_texture.h"
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

Light_Sample Environment_Light_Sampler::sample(Vector2 u) const {
    float pdf_uv;
    Vector2 uv = radiance_distribution.sample(u, &pdf_uv);
    ASSERT(pdf_uv != 0.f);

    float phi = uv[0] * Pi2;
    float theta = uv[1] * Pi;

    float sin_theta = std::sin(theta);

    // pdf transformation from uv to solid angle measure
    float pdf = transform_pdf_uv_to_solid_angle_measure(pdf_uv, sin_theta);

    Vector3 wi = { sin_theta * std::cos(phi), sin_theta * std::sin(phi), std::cos(theta) };
    wi = transform_vector(light->light_to_world, wi);

    ColorRGB Le = environment_map->sample_bilinear(uv, 0, Wrap_Mode::clamp) * light->scale;

    return Light_Sample{ wi, Le, pdf };
}

ColorRGB Environment_Light_Sampler::get_radiance_for_direction(const Vector3& world_direction) const {
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
