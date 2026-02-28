#ifdef __cplusplus
    #pragma once
    #define SLANG_PUBLIC
    namespace GPU_Types {
#else
    module shared_material;
    #define SLANG_PUBLIC public
#endif

SLANG_PUBLIC struct Lambertian_Material {
    SLANG_PUBLIC float r, g, b; // albedo in [0, 1] range
    SLANG_PUBLIC int albedo_texture_index;
    SLANG_PUBLIC float u_scale;
    SLANG_PUBLIC float v_scale;
};

#ifdef __cplusplus
} // namespace GPU_Types
#endif
