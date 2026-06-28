#pragma once

#include "sampling.h"
#include "lib/color.h"
#include "lib/light.h"

struct Environment_Light;
struct Diffuse_Sphere_Light;
struct Diffuse_Triangle_Mesh_Light;
struct Intersection;
class Image_Texture;

struct Environment_Light_Sampler {
    const Environment_Light* light = nullptr;
    const Image_Texture* environment_map = nullptr;
    Distribution_2D radiance_distribution;

    bool initialized() const { return light != nullptr; }
    ColorRGB sample(Vector2 u, Vector3* wi, float* pdf) const;
    ColorRGB get_unfiltered_radiance_for_direction(const Vector3& world_direction) const;
    ColorRGB get_filtered_radiance_for_direction(const Vector3& world_direction) const;
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

    // Uniformly samples solid angle formed by the shading point and a sphere.
    // Returns position on the sphere that corresponds to the sampled direction.
    Vector3 sample(Vector2 u) const;

    bool is_direction_inside_light_cone(const Vector3& wi) const;
};

struct Diffuse_Triangle_Mesh_Light_Sampler {
    const Diffuse_Triangle_Mesh_Light* light = nullptr;
    const Triangle_Mesh* mesh = nullptr;
    float mesh_area = 0.f;
    Distribution_1D triangle_distribution; // this pdf is proportional to triangle area

    // Samples point on the light source and returns it.
    // The pdf is computed with regard to solid angle measure.
    Vector3 sample(Vector2 u, const Vector3& shading_pos, float* pdf) const;

    float pdf(const Vector3& shading_pos, const Vector3& wi, const Intersection& light_intersection) const;
};
