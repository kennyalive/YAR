#pragma once

#include "lib/color.h"
#include "lib/triangle_mesh.h"
#include "lib/vector.h"

enum class Light_Type : uint32_t {
    point, // delta light
    spot, // delta light
    directional, // delta light
    diffuse_rectangular,
    diffuse_sphere,
    diffuse_triangle_mesh,
    environment_map,
    count,
    null_light = std::numeric_limits<uint32_t>::max()
};

struct Light_Handle {
    Light_Type type = Light_Type::null_light;
    int index = -1;

    bool operator==(Light_Handle h) const {
        return type == h.type && index == h.index;
    }
    bool operator!=(Light_Handle h) const {
        return !(*this == h);
    }
};

static_assert(sizeof(Light_Handle) == 8);
constexpr Light_Handle Null_Light = { Light_Type::null_light, -1 };
constexpr int Light_Type_Count = static_cast<int>(Light_Type::count);

struct Point_Light {
    Vector3 position;
    ColorRGB intensity;
};

struct Spot_Light {
    Vector3 position;
    Vector3 direction;
    float cone_angle = 0.f;
    float penumbra_angle = 0.f;
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
    int sample_count = 1;

    Triangle_Mesh get_geometry() const;
};

struct Diffuse_Sphere_Light {
    Vector3 position;
    ColorRGB emitted_radiance;
    float radius = 0.f;
    int sample_count = 0;
};

struct Diffuse_Triangle_Mesh_Light {
    Matrix3x4 light_to_world_transform;
    ColorRGB emitted_radiance;
    uint32_t triangle_mesh_index = 0;
};

struct Environment_Light {
    Matrix3x4 light_to_world;
    Matrix3x4 world_to_light;
    ColorRGB scale = ColorRGB(1);
    int environment_map_index = -1;
    int sample_count = 0;
};

struct Lights {
    std::vector<Point_Light> point_lights;
    std::vector<Spot_Light> spot_lights;
    std::vector<Directional_Light> directional_lights;
    std::vector<Diffuse_Rectangular_Light> diffuse_rectangular_lights;
    std::vector<Diffuse_Sphere_Light> diffuse_sphere_lights;
    std::vector<Diffuse_Triangle_Mesh_Light> diffuse_triangle_mesh_lights;

    Environment_Light environment_light;
    bool has_environment_light = false;

    bool has_lights() const;
    void update_total_light_count();

    int total_light_count = 0;
};
