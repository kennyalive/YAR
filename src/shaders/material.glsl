const int Material_None = 0;
const int Material_Lambertian = 1;

struct Material_Handle {
    int type;
    int index;
};

struct Lambertian_Material {
    float r, g, b; // albedo in [0, 1] range
};

layout(std430, set=1, binding=0) readonly buffer Lambertian_Material_Buffer {
    Lambertian_Material lambertian_materials[];
};

vec3 compute_bsdf(Material_Handle mtl_handle, vec3 wi, vec3 wo) {
    if (mtl_handle.type == Material_Lambertian) {
        Lambertian_Material mtl = lambertian_materials[mtl_handle.index];
        vec3 albedo = vec3(mtl.r, mtl.g, mtl.b);
        return Pi_Inv * albedo;
    }
    return vec3(1, 0, 0);    
}

