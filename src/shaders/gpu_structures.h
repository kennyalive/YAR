#ifdef __cplusplus
    #pragma once
    #include "vk.h"
    #include "lib/color.h"
    #include "lib/io.h"
    #include "lib/vector.h"
#else
    #define ColorRGB vec3
    #define Vector2 vec2
    #define Vector3 vec3
    #define Matrix3x4 mat4x3
#endif

#ifdef __cplusplus
namespace GPU_Types {
#endif

struct Mesh_Material {
    ColorRGB    k_diffuse;
    float       pad0;
    ColorRGB    k_specular;
    float       pad1;
};

struct Point_Light {
    Vector3     position;
    float       pad0;
    ColorRGB    intensity;
    float       pad1;
};

struct Diffuse_Rectangular_Light {
    Matrix3x4   light_to_world;

    ColorRGB    emitted_radiance;
    float       pad0;

    Vector2     size;
    float       area;
    int         shadow_ray_count;

#ifdef __cplusplus
    void init(const RGB_Diffuse_Rectangular_Light_Data& data) {
        light_to_world      = data.light_to_world_transform;

        emitted_radiance    = data.emitted_radiance;
        pad0                = 0.f;

        size                = data.size;
        area                = data.size.x * data.size.y;
        shadow_ray_count    = data.shadow_ray_count;
    }
#endif
};

#ifdef __cplusplus
} // namespace GPU_Types
#endif
