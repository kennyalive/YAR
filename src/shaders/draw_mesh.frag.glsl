#version 460
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_nonuniform_qualifier : require

#include "common.glsl"
#include "geometry.glsl"
#include "material_resources.glsl"
#include "light_resources.glsl"
#include "compute_bsdf.glsl"
#include "shared_main.h"

struct Frag_In {
    vec3 normal;
    vec3 pos;
    vec2 uv;
};

layout(push_constant) uniform Push_Constants {
    int instance_index;
};

layout(location=0) in Frag_In frag_in;

layout(location = 0) out vec4 color_attachment0;

layout(std140, set=KERNEL_SET_0, binding=0) uniform Global_Uniform_Block {
    mat4x4 model_view_proj;
    mat4x4 model_view;
    mat4x4 view;
    int point_light_count;
    int directional_light_count;
    int diffuse_rectangular_light_count;
    float pad0;
};

void main() {
    vec3 n = normalize(frag_in.normal);
    vec3 L = vec3(0);

    Instance_Info instance_info = instance_infos[instance_index];

    Material_Handle mtl_handle = instance_info.material;

    vec3 wo = normalize(-frag_in.pos);

    for (int i = 0; i < point_light_count; i++) {
        vec3 light_pos_eye = vec3(view * vec4(point_lights[i].position, 1.0));
        vec3 light_vec = light_pos_eye - frag_in.pos;
        float light_dist_sq_inv = 1.f / dot(light_vec, light_vec);
        vec3 light_dir = light_vec * sqrt(light_dist_sq_inv);

        vec3 bsdf = compute_bsdf(mtl_handle, frag_in.uv, light_dir, wo);
        L += bsdf * point_lights[i].intensity * (light_dist_sq_inv * max(0, dot(n, light_dir)));
    }

    for (int i = 0; i < directional_light_count; i++) {
        vec3 light_dir = vec3(view * vec4(directional_lights[i].direction, 0.0));

        vec3 bsdf = compute_bsdf(mtl_handle, frag_in.uv, light_dir, wo);
        L += bsdf * directional_lights[i].irradiance * max(0, dot(n, light_dir));
    }

    uint seed = uint(gl_FragCoord.y)*uint(800) + uint(gl_FragCoord.x);
    uint rng_state = wang_hash(seed);

    for (int i = 0; i < diffuse_rectangular_light_count; i++) {
        Diffuse_Rectangular_Light light = diffuse_rectangular_lights[i];
        if (light.shadow_ray_count == 0)
            continue;
        vec3 L2 = vec3(0);
        for (int k = 0; k < light.shadow_ray_count; k++) {
            vec2 u;
            u.x = float(rng_state) * (1.0/float(0xffffffffu));
            rng_state = rand_xorshift(rng_state);
            u.y = float(rng_state) * (1.0/float(0xffffffffu));

            vec3 local_light_point = vec3(light.size.x/2.0 * u.x, light.size.y/2.0 * u.y, 0.f);
            vec3 light_point = light.light_to_world_transform * vec4(local_light_point, 1.0);
            vec3 light_point_eye = vec3(view * vec4(light_point, 1.0));

            vec3 light_vec = light_point_eye - frag_in.pos;
            float light_dist = length(light_vec);
            vec3 light_dir = light_vec / light_dist;

            vec3 light_normal = vec3(model_view * vec4(light.light_to_world_transform[2], 0.0));
            float light_n_dot_l = dot(light_normal, -light_dir);
            if (light_n_dot_l <= 0.f)
                continue;

            float n_dot_l = dot(n, light_dir);
            if (n_dot_l <= 0.f)
                continue;

            vec3 bsdf = compute_bsdf(mtl_handle, frag_in.uv, light_dir, wo);
            L2 += light.area * light.emitted_radiance * bsdf * (n_dot_l * light_n_dot_l / (light_dist * light_dist));
        }
        L2 /= float(light.shadow_ray_count);
        L += L2;
    }

    if (instance_info.area_light_index != -1) {
        L += diffuse_rectangular_lights[instance_info.area_light_index].emitted_radiance;
    }

    color_attachment0 = vec4(srgb_encode(L), 1);
}
