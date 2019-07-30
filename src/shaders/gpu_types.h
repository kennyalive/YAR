#ifdef __cplusplus
    #pragma once
    #include "vk.h"
    #include "lib/color.h"
    #include "lib/light.h"
    #include "lib/material.h"
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

struct Point_Light {
    Vector3     position;
    float       pad0;
    ColorRGB    intensity;
    float       pad1;

#ifdef __cplusplus
    void init(const ::Point_Light& data) {
        position    = data.position;
        pad0        = 0.f;

        intensity   = data.intensity;
        pad1        = 0.f;
    }
#endif
};

struct Diffuse_Rectangular_Light {
    Matrix3x4   light_to_world_transform;

    ColorRGB    emitted_radiance;
    float       pad0;

    Vector2     size;
    float       area;
    int         shadow_ray_count;

#ifdef __cplusplus
    void init(const ::Diffuse_Rectangular_Light& data) {
        light_to_world_transform = data.light_to_world_transform;

        emitted_radiance    = data.emitted_radiance;
        pad0                = 0.f;

        size                = data.size;
        area                = data.size.x * data.size.y;
        shadow_ray_count    = data.shadow_ray_count;
    }
#endif
};

struct Instance_Info {
    Material_Handle material;
    int area_light_index; // -1 if not an area light
};

#ifdef __cplusplus
} // namespace GPU_Types
#endif
