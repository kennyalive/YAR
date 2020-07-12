#pragma once

#include "sampling.h"
#include "lib/color.h"

struct Environment_Light;
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
};
