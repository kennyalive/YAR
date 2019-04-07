#version 460
#extension GL_GOOGLE_include_directive : require
#extension GL_NV_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : require

#include "common.glsl"

#define HIT_SHADER
#include "rt_utils.glsl"

hitAttributeNV vec2 attribs;

struct Mesh_Vertex {
    float x, y, z;
    float nx, ny, nz;
    float u, v;
};

layout (location=0) rayPayloadInNV Ray_Payload payload;
layout (location=1) rayPayloadNV Shadow_Ray_Payload shadow_ray_payload;

layout(set=0, binding = 1) uniform accelerationStructureNV accel;

layout(std140, binding=2) uniform Uniform_Block {
    mat4x3  camera_to_world;
    int     point_light_count;
    int     diffuse_rectangular_light_count;
    vec2    pad0;
};

layout(std430, binding=3) readonly buffer Index_Buffer {
    uint indices[];
} index_buffers[];

layout(std430, binding=4) readonly buffer Vertex_Buffer {
    Mesh_Vertex vertices[];
} vertex_buffers[];

struct Material {
    vec3    k_diffuse;
    float   pad0;
    vec3    k_specular;
    float   pad1;
};

layout(std430, binding=5) readonly buffer Materials {
    Material materials[];
};

layout(std430, binding=6) readonly buffer Point_Light_Buffer {
    Point_Light point_lights[];
};

layout(std430, binding=7) readonly buffer Diffuse_Rectangular_Light_Buffer {
    Diffuse_Rectangular_Light diffuse_rectangular_lights[];
};

Vertex fetch_vertex(int index) {
    uint vertex_index = index_buffers[nonuniformEXT(gl_InstanceCustomIndexNV)].indices[index];
    Mesh_Vertex bv = vertex_buffers[nonuniformEXT(gl_InstanceCustomIndexNV)].vertices[vertex_index];

    Vertex v;
    v.p = vec3(bv.x, bv.y, bv.z);
    v.n = vec3(bv.nx, bv.ny, bv.nz);
    v.uv = fract(vec2(bv.u, bv.v));
    return v;
}

void main() {
    Vertex v0 = fetch_vertex(gl_PrimitiveID*3 + 0);
    Vertex v1 = fetch_vertex(gl_PrimitiveID*3 + 1);
    Vertex v2 = fetch_vertex(gl_PrimitiveID*3 + 2);

    mat3 normal_transform = mat3(gl_ObjectToWorldNV);
    vec3 n0 = normal_transform * v0.n;
    vec3 n1 = normal_transform * v1.n;
    vec3 n2 = normal_transform * v2.n;
    vec3 n = normalize(barycentric_interpolate(attribs.x, attribs.y, n0, n1, n2));

    vec3 L = vec3(0);
    for (int i = 0; i < point_light_count; i++) {
        vec3 p = gl_WorldRayOriginNV + gl_WorldRayDirectionNV * gl_HitTNV + 1e-3*n;
        vec3 light_vec = point_lights[i].position - p;
        float light_dist = length(light_vec);
        vec3 light_dir = light_vec / light_dist;

        float n_dot_l = dot(n, light_dir);
        if (n_dot_l <= 0.0)
            continue;

        // trace shadow ray
        const float tmin = 1e-3;
        shadow_ray_payload.shadow_factor = 1.0f;
        traceNV(accel, gl_RayFlagsOpaqueNV|gl_RayFlagsTerminateOnFirstHitNV, 0xff, 1, 0, 1, p, tmin, light_dir, light_dist - tmin, 1);
        if (shadow_ray_payload.shadow_factor == 0.0)
            continue;

        vec3 irradiance = point_lights[i].intensity * (n_dot_l / (light_dist * light_dist));
        vec3 bsdf = materials[gl_InstanceCustomIndexNV].k_diffuse * Pi_Inv;
        L += shadow_ray_payload.shadow_factor * irradiance * bsdf;
    }

    uint seed = uint(gl_LaunchIDNV.y)*uint(800) + uint(gl_LaunchIDNV.x);
    uint rng_state = wang_hash(seed);

    for (int i = 0; i < diffuse_rectangular_light_count; i++) {
        Diffuse_Rectangular_Light light = diffuse_rectangular_lights[i];
        for (int k = 0; k < light.shadow_ray_count; k++) {
            vec2 u;
            u.x = float(rng_state) * (1.0/float(0xffffffffu));
            rng_state = rand_xorshift(rng_state);
            u.y = float(rng_state) * (1.0/float(0xffffffffu));

            vec3 local_light_point = vec3(light.size.x/2.0 * u.x, light.size.y/2.0 * u.y, 0.f);
            vec3 light_point = light.light_to_world * vec4(local_light_point, 1.0);

            vec3 p = gl_WorldRayOriginNV + gl_WorldRayDirectionNV * gl_HitTNV + 1e-3*n;

            vec3 light_vec = light_point - p;
            float light_dist = length(light_vec);
            vec3 light_dir = light_vec / light_dist;

            vec3 light_normal = light.light_to_world[2];
            float light_n_dot_l = dot(light_normal, -light_dir);
            if (light_n_dot_l <= 0.f)
                continue;

            float n_dot_l = dot(n, light_dir);
            if (n_dot_l <= 0.f)
                continue;

            // trace shadow ray
            const float tmin = 1e-3;
            shadow_ray_payload.shadow_factor = 1.0f;
            traceNV(accel, gl_RayFlagsOpaqueNV|gl_RayFlagsTerminateOnFirstHitNV, 0xff, 1, 0, 1, p, tmin, light_dir, light_dist - tmin, 1);
            if (shadow_ray_payload.shadow_factor == 0.0)
                continue;

            vec3 bsdf = materials[gl_InstanceCustomIndexNV].k_diffuse * Pi_Inv;

            L += shadow_ray_payload.shadow_factor * bsdf * light.area * light.emitted_radiance * (n_dot_l * light_n_dot_l / (light_dist * light_dist));
        }
        L /= float(light.shadow_ray_count);
    }

    payload.color = srgb_encode(vec3(L));
}
