#ifdef __cplusplus
    #pragma once
    #include "shared_main.slang"
    namespace GPU_Types {
#else
    module shared_light;
    import shared_main;
    #define SLANG_PUBLIC public
#endif

SLANG_PUBLIC struct Point_Light {
    SLANG_PUBLIC Vector3 position;
    float pad0;
    SLANG_PUBLIC ColorRGB intensity;
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

SLANG_PUBLIC struct Directional_Light {
    SLANG_PUBLIC Vector3 direction;
    float pad0;
    SLANG_PUBLIC ColorRGB irradiance;
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

SLANG_PUBLIC struct Diffuse_Rectangular_Light {
    SLANG_PUBLIC Matrix3x4 light_to_world_transform;

    SLANG_PUBLIC ColorRGB emitted_radiance;
    float pad0;

    SLANG_PUBLIC Vector2 size;
    SLANG_PUBLIC float area;
    SLANG_PUBLIC int shadow_ray_count;

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
