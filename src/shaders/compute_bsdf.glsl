#include "base_resources.glsl"

vec3 compute_bsdf(Material_Handle mtl_handle, vec2 uv, vec3 wi, vec3 wo) {
    if (mtl_handle.type == Material_Lambertian) {
        Lambertian_Material mtl = lambertian_materials[mtl_handle.index];

        vec3 albedo;
        if (mtl.albedo_texture_index > 0)
            albedo = texture(sampler2D(images_2d[mtl.albedo_texture_index], image_sampler), uv).xyz;
        else
            albedo = vec3(mtl.r, mtl.g, mtl.b);

        return Pi_Inv * albedo;
    }
    return vec3(1, 0, 0);    
}

