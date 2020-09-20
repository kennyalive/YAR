#pragma once

#include "sampling.h"
#include "lib/color.h"
#include "lib/light.h"
#include "lib/math.h"
#include "lib/random.h"

struct Environment_Light;
struct Diffuse_Sphere_Light;
struct Scene_Context;
class Image_Texture;

struct Light_Sample {
    Vector3 Wi;
    ColorRGB Le;
    float pdf = 0.f;
};

struct Environment_Light_Sampler {
    const Environment_Light* light = nullptr;
    const Image_Texture* environment_map = nullptr;
    Distribution_2D radiance_distribution;

    Light_Sample sample(Vector2 u) const;
    ColorRGB get_radiance_for_direction(const Vector3& world_direction) const;
    float pdf(const Vector3& world_direction) const;
};

struct Diffuse_Sphere_Light_Sampler {
    const Diffuse_Sphere_Light& light;
    Vector3 shading_pos;

    // coordinate system formed by the direction (light position -> shading point) as z axis
    // and two other orthogonal axes are choosen arbitrarily
    Vector3 axes[3];

    float d_center = 0.f; // distance from shading_pos to light center
    float cos_theta_max = 0.f;
    float cone_sampling_pdf = 0.f;

    Diffuse_Sphere_Light_Sampler(
        const Diffuse_Sphere_Light& light,
        const Vector3& shading_pos);

    // Returns distance to the sampled point on the sphere.
    // Output parameter wi - direction from the shading position to the sampled point
    float sample(Vector2 u, Vector3* wi) const;

    bool is_direction_inside_light_cone(const Vector3& wi) const;
};
