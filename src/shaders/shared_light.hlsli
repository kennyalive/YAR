#ifndef SHARED_LIGHT_HLSL
#define SHARED_LIGHT_HLSL

#include "shared_main.hlsli"

#ifdef __cplusplus
namespace GPU_Types {
#endif

struct Point_Light {
    Vector3 position;
    float pad0;
    ColorRGB intensity;
    float pad1;

#ifdef __cplusplus
    void init(const ::Point_Light& data) {
        position    = data.position;
        pad0        = 0.f;

        intensity   = data.intensity;
        pad1        = 0.f;
    }
#endif
};

struct Directional_Light {
    Vector3 direction;
    float pad0;
    ColorRGB irradiance;
    float pad1;

#ifdef __cplusplus
    void init(const ::Directional_Light& data) {
        direction = data.direction;
        pad0 = 0.f;
        irradiance = data.irradiance;
        pad1 = 0.f;
    }
#endif
};

struct Diffuse_Rectangular_Light {
    Matrix3x4 light_to_world_transform;

    ColorRGB emitted_radiance;
    float pad0;

    Vector2 size;
    float area;
    int shadow_ray_count;

#ifdef __cplusplus
    void init(const ::Diffuse_Rectangular_Light& data) {
        light_to_world_transform = data.light_to_world_transform;

        emitted_radiance    = data.emitted_radiance;
        pad0                = 0.f;

        size                = data.size;
        area                = data.size.x * data.size.y;
        shadow_ray_count    = data.sample_count;
    }
#endif
};

#ifdef __cplusplus
} // namespace GPU_Types
#endif

#endif
