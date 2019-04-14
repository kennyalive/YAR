#pragma once

#include "material.h"

#include "lib/color.h"
#include "lib/matrix.h"
#include "lib/random.h"
#include "lib/vector.h"

struct Local_Geometry;
struct Render_Context;
struct RGB_Diffuse_Rectangular_Light_Data;

struct Point_Light {
    Vector3 position;
    ColorRGB intensity;
};

struct Diffuse_Rectangular_Light {
    Matrix3x4 light_to_world_transform;
    ColorRGB emitted_radiance;
    Vector2 size;
    float area;
    int shadow_ray_count;

    Diffuse_Rectangular_Light(const RGB_Diffuse_Rectangular_Light_Data& light_data);
};

enum class Light_Type : uint32_t {
    none,
    point_light,
    diffuse_rectangular,
};

struct Light_Handle {
    Light_Type type;
    int index;

    bool operator==(Light_Handle h) const {
        return type == h.type && index == h.index;
    }
    bool operator!=(Light_Handle h) const {
        return !(*this == h);
    }
};
static_assert(sizeof(Light_Handle) == 8);
constexpr Light_Handle Null_Light = { Light_Type::none, -1 };

ColorRGB compute_direct_lighting(
    const Render_Context& ctx,
    const Local_Geometry& local_geom,
    const Vector3& wo,
    Material_Handle material,
    pcg32_random_t* rng);
