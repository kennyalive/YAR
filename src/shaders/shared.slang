// Contains definitions that are shared between shader and cpp code.
// For shared structures this ensures that CPU and GPU sees the same memory layout.

#ifdef __cplusplus
    #pragma once
    #include "../lib/color.h"
    #include "../lib/geometry.h"
    #include "../lib/light.h"
    #include "../lib/material.h"
    #include "../lib/vector.h"
    #define SLANG_PUBLIC
    using uint = uint32_t;
#else
    module shared;
    public typealias ColorRGB = float3;
    public typealias Vector2 = float2;
    public typealias Vector3 = float3;
    public typealias Matrix3x4 = float3x4;
    #define SLANG_PUBLIC public
#endif

// Bindings configuration.
SLANG_PUBLIC static const int BASE_SET_INDEX = 0;
static const int BASE_SET_BINDING_TEXTURES = 0;
static const int BASE_SET_BINDING_SAMPLER = 1;
SLANG_PUBLIC static const int BASE_SET_BINDING_INSTANCE_INFO = 2;

static const int MATERIAL_SET_INDEX = 1;
static const int LAMBERTIAN_MATERIAL_BINDING = 0;

SLANG_PUBLIC static const int LIGHT_SET_INDEX = 2;
SLANG_PUBLIC static const int POINT_LIGHT_BINDING = 0;
SLANG_PUBLIC static const int DIRECTIONAL_LIGHT_BINDING = 1;
SLANG_PUBLIC static const int DIFFUSE_RECTANGULAR_LIGHT_BINDING = 2;

SLANG_PUBLIC static const int KERNEL_SET_0 = 3;

// Default texture indices.
SLANG_PUBLIC static const int Black_2D_Texture_Index = 0;
static const int White_2D_Texture_Index = 1;
SLANG_PUBLIC static const int Predefined_Texture_Count = 2;

//===========================================================
// Shared structures.
// C++ definitions are in the GPU_Types namespace.
//===========================================================
#ifdef __cplusplus
namespace GPU_Types {
#endif

//
// Main types
//
SLANG_PUBLIC struct Geometry_Handle {
    int type;
    SLANG_PUBLIC int index;
#ifdef __cplusplus
    void init(const ::Geometry_Handle& data) {
        type = static_cast<int>(data.type);
        index = data.index;
    }
#endif
};

SLANG_PUBLIC struct Material_Handle {
    SLANG_PUBLIC int type;
    SLANG_PUBLIC int index;
#ifdef __cplusplus
    void init(const ::Material_Handle& data) {
        type = static_cast<int>(data.type);
        index = data.index;
    }
#endif
};

struct Light_Handle {
    int type;
    int index;
#ifdef __cplusplus
    void init(const ::Light_Handle& data) {
        type = static_cast<int>(data.type);
        index = data.index;
    }
#endif
};

SLANG_PUBLIC struct Instance_Info {
    SLANG_PUBLIC Material_Handle material;
    SLANG_PUBLIC Geometry_Handle geometry;

    SLANG_PUBLIC int area_light_index; // -1 if not an area light
    float pad0;
    float pad1;
    float pad2;
   
    Matrix3x4 object_to_world_transform;
};

//
// Global scene information
//
SLANG_PUBLIC struct Scene_Info
{
    SLANG_PUBLIC uint point_light_count;
    SLANG_PUBLIC uint directional_light_count;
    SLANG_PUBLIC uint rect_light_count;
};

//
// Materials
//
SLANG_PUBLIC struct Lambertian_Material {
    SLANG_PUBLIC float r, g, b; // albedo in [0, 1] range
    SLANG_PUBLIC int albedo_texture_index;
    SLANG_PUBLIC float u_scale;
    SLANG_PUBLIC float v_scale;
};

//
// Lights
//
SLANG_PUBLIC struct Point_Light {
    SLANG_PUBLIC Vector3 position;
    float pad0;
    SLANG_PUBLIC ColorRGB intensity;
    float pad1;

#ifdef __cplusplus
    void init(const ::Point_Light &data) {
        position = data.position;
        pad0 = 0.f;

        intensity = data.intensity;
        pad1 = 0.f;
    }
#endif
};

SLANG_PUBLIC struct Directional_Light {
    SLANG_PUBLIC Vector3 direction;
    float pad0;
    SLANG_PUBLIC ColorRGB irradiance;
    float pad1;

#ifdef __cplusplus
    void init(const ::Directional_Light &data) {
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
    void init(const ::Diffuse_Rectangular_Light &data) {
        light_to_world_transform = data.light_to_world_transform;

        emitted_radiance = data.emitted_radiance;
        pad0 = 0.f;

        size = data.size;
        area = data.size.x * data.size.y;
        shadow_ray_count = data.sample_count;
    }
#endif
};

#ifdef __cplusplus
} // namespace GPU_Types
#endif
