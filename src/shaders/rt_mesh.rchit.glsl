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

struct Point_Light {
    vec3 position;
    vec3 intensity;
};

layout (location=0) rayPayloadInNV Ray_Payload payload;

layout(std140, binding=2) uniform Uniform_Block {
    mat4x3 camera_to_world;
    Point_Light point_lights[8];
    int point_light_count;
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
        vec3 p = gl_WorldRayOriginNV + gl_WorldRayDirectionNV * gl_HitTNV;
        vec3 light_vec = point_lights[i].position - p;
        float light_dist_sq_inv = 1.f / dot(light_vec, light_vec);
        vec3 light_dir = light_vec * sqrt(light_dist_sq_inv);

        L += (materials[gl_InstanceCustomIndexNV].k_diffuse * Pi_Inv) * point_lights[i].intensity * (light_dist_sq_inv * max(0, dot(n, light_dir)));
    }


    payload.color = srgb_encode(vec3(L));
}
