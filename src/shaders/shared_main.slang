// shared_xxx.h files contain definitions that are shared between
// shader and cpp code. For shared structures this ensures that
// CPU and GPU sees the same memory layout.

#ifdef __cplusplus
    #pragma once
    #include "../lib/color.h"
    #include "../lib/geometry.h"
    #include "../lib/light.h"
    #include "../lib/material.h"
    #include "../lib/vector.h"
    #define SLANG_PUBLIC
#else
    module shared_main;
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

#ifdef __cplusplus
namespace GPU_Types {
#endif

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

#ifdef __cplusplus
} // namespace GPU_Types
#endif
