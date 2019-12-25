#pragma once

#include "lib/color.h"
#include "lib/triangle_mesh.h"
#include "lib/vector.h"

enum class Light_Type : uint32_t {
    none,
    point,
    diffuse_rectangular,
};

struct Light_Handle {
    Light_Type type = Light_Type::none;
    int index = -1;

    bool operator==(Light_Handle h) const {
        return type == h.type && index == h.index;
    }
    bool operator!=(Light_Handle h) const {
        return !(*this == h);
    }
};

static_assert(sizeof(Light_Handle) == 8);
constexpr Light_Handle Null_Light = { Light_Type::none, -1 };

struct Point_Light {
    Vector3 position;
    ColorRGB intensity;
};

struct Diffuse_Rectangular_Light {
    Matrix3x4 light_to_world_transform;
    ColorRGB emitted_radiance;
    Vector2 size;
    int shadow_ray_count;

    Triangle_Mesh get_geometry() const;
};

struct Lights {
    std::vector<Point_Light> point_lights;
    std::vector<Diffuse_Rectangular_Light> diffuse_rectangular_lights;
};

