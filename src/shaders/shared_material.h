#ifdef __cplusplus
namespace GPU_Types {
#endif

struct Lambertian_Material {
    float r, g, b; // albedo in [0, 1] range
    int albedo_texture_index;
};

#ifdef __cplusplus
} // namespace GPU_Types
#endif

