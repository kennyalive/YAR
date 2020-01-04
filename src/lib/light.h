#pragma once

#include "lib/color.h"
#include "lib/triangle_mesh.h"
#include "lib/vector.h"

enum class Light_Type : uint32_t {
    none,
    point, // delta light
    directional, // delta light
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

struct Directional_Light {
    Vector3 direction;

    // Emission from a directional light is defined by the irradiance it
    // creates on the surface that is perpendicular to direction vector.
    //
    // The decision was made not to use radiance for directional lights.
    // Irradicance captures non-physical behavior of dirrectional light better
    // than radiance, because radiance should be represented as delta function
    // (usually implicitly in the code) which makes it's harder to reason about
    // and in case of irradiance we don't need any additional extensions and 
    // implicit conventions.
    //
    // The reflected radiance due to directional light is computed as:
    // L(wo) = F(wo, light_dir) * E * abs(cos(N, ligt_dir))
    // where
    // E - directional light's irradiance
    // F - BSDF
    ColorRGB irradiance;
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
    std::vector<Directional_Light> directional_lights;
    std::vector<Diffuse_Rectangular_Light> diffuse_rectangular_lights;

    void append(const Lights& lights);
};
