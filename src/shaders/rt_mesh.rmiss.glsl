#version 460
#extension GL_GOOGLE_include_directive : require
#extension GL_NV_ray_tracing : require

#include "common.glsl"
#include "rt_utils.glsl"

layout(std140, binding=2) uniform Uniform_Block {
    mat4x3  camera_to_world;
    int     point_light_count;
    int     diffuse_rectangular_light_count;
    vec2    pad0;
};

layout(std430, binding=6) readonly buffer Diffuse_Rectangular_Light_Buffer {
    Diffuse_Rectangular_Light diffuse_rectangular_lights[];
};

layout (location=0) rayPayloadInNV Ray_Payload payload;

mat4x3 get_inverted_transform(mat4x3 m) {
    vec3 x_axis = m[0];
    vec3 y_axis = m[1];
    vec3 z_axis = m[2];
    vec3 origin = m[3];
    mat4x3 inv_m;
    inv_m[0] = vec3(x_axis.x, y_axis.x, z_axis.x);
    inv_m[1] = vec3(x_axis.y, y_axis.y, z_axis.y);
    inv_m[2] = vec3(x_axis.z, y_axis.z, z_axis.z);
    inv_m[3] = vec3(-dot(x_axis, origin), -dot(y_axis, origin), -dot(z_axis, origin));
    return inv_m;
}

bool is_ray_intersects_rectangular_light(Diffuse_Rectangular_Light light) {
    mat4x3 world_to_light = get_inverted_transform(light.light_to_world);
    vec3 ray_o = world_to_light * vec4(gl_WorldRayOriginNV, 1.f);
    vec3 ray_d = world_to_light * vec4(gl_WorldRayDirectionNV, 0.f);

    if (ray_d.z >= 0.f)
        return false;

    float t = -ray_o.z / ray_d.z;
    vec3 p = ray_o + t * ray_d;
    return abs(p.x) <= light.size.x/2.0 && abs(p.y) <= light.size.y/2.0;
}

vec3 compute_self_emitted_radiance() {
    vec3 Le = vec3(0);
    for (int i = 0; i < diffuse_rectangular_light_count; i++) {
        Diffuse_Rectangular_Light light = diffuse_rectangular_lights[i];
        if (is_ray_intersects_rectangular_light(light))
            Le += light.emitted_radiance;
    }
    return Le;
}

void main() {
    payload.color = srgb_encode(compute_self_emitted_radiance());
}
