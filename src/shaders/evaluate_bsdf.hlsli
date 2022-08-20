#ifndef EVALUATE_BSDF_HLSL
#define EVALUATE_BSDF_HLSL

#include "base_resources.hlsli"
#include "common.hlsli"
#include "material_resources.hlsli"
#include "shared_main.hlsli"

float3 evaluate_bsdf(Material_Handle mtl_handle, float2 uv, float3 wi, float3 wo)
{
    if (mtl_handle.type == Material_Lambertian) {
        Lambertian_Material mtl = lambertian_materials[mtl_handle.index];
        float3 albedo;
        if (mtl.albedo_texture_index > 0) {
            float2 uv_scaled = uv * float2(mtl.u_scale, mtl.v_scale);
            albedo = images_2d[mtl.albedo_texture_index].SampleLevel(image_sampler, uv_scaled, 0).xyz;
        }
        else {
            albedo = float3(mtl.r, mtl.g, mtl.b);
        }
        return Pi_Inv * albedo;
    }
    return float3(1, 0, 0);
}

#endif
