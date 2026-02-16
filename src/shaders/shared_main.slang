#ifndef SHARED_MAIN_HLSL
#define SHARED_MAIN_HLSL

// shared_xxx.h files contain definitions that are shared between
// shader and cpp code. For shared structures this ensures that
// CPU and GPU sees the same memory layout.
// shared_main.h should be included before any other shared_xxx.h

#ifdef __cplusplus
    #pragma once
    #include "../lib/color.h"
    #include "../lib/geometry.h"
    #include "../lib/light.h"
    #include "../lib/material.h"
    #include "../lib/vector.h"
#else
    #define ColorRGB float3
    #define Vector2 float2
    #define Vector3 float3
    #define Matrix3x4 float3x4
#endif

// Bindings configuration.
#ifndef BASE_SET_INDEX 
#define BASE_SET_INDEX 0
#endif
static const int BASE_SET_BINDING_TEXTURES = 0;
static const int BASE_SET_BINDING_SAMPLER = 1;
static const int BASE_SET_BINDING_INSTANCE_INFO = 2;

#ifndef MATERIAL_SET_INDEX
#define MATERIAL_SET_INDEX 1
#endif
static const int LAMBERTIAN_MATERIAL_BINDING = 0;

#ifndef LIGHT_SET_INDEX
#define LIGHT_SET_INDEX 2
#endif
static const int POINT_LIGHT_BINDING = 0;
static const int DIRECTIONAL_LIGHT_BINDING = 1;
static const int DIFFUSE_RECTANGULAR_LIGHT_BINDING = 2;

#define KERNEL_SET_0 3

// Default texture indices.
static const int Black_2D_Texture_Index = 0;
static const int White_2D_Texture_Index = 1;
static const int Predefined_Texture_Count = 2;

#ifdef __cplusplus
namespace GPU_Types {
#endif

struct Geometry_Handle {
    int type;
    int index;
#ifdef __cplusplus
    void init(const ::Geometry_Handle& data) {
        type = static_cast<int>(data.type);
        index = data.index;
    }
#endif
};

struct Material_Handle {
    int type;
    int index;
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

struct Instance_Info {
    Material_Handle material;
    Geometry_Handle geometry;
    
    int area_light_index; // -1 if not an area light
    float pad0;
    float pad1;
    float pad2;
   
    Matrix3x4 object_to_world_transform;
};

#ifdef __cplusplus
} // namespace GPU_Types
#endif

#endif
