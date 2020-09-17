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

struct Diffuse_Sphere_Light_Area_Sampler {
    const Scene_Context* scene_ctx = nullptr;
    pcg32_random_t* rng = nullptr;
    Vector3 shading_pos;
    Light_Handle light_handle;
    Vector3 sphere_center;
    float area_pdf = 0.f;

    Diffuse_Sphere_Light_Area_Sampler(
        const Scene_Context* scene_ctx,
        pcg32_random_t* rng,
        const Vector3& shading_pos,
        int light_index);

    Vector3 sample_direction_on_sphere();
    float pdf(const Vector3& wi) const;
};
