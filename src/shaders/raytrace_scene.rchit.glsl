#version 460
#extension GL_GOOGLE_include_directive : require
#extension GL_NV_ray_tracing : require
#extension GL_EXT_nonuniform_qualifier : require

#include "common.glsl"
#include "shared_main.h"

#define HIT_SHADER
#include "rt_utils.glsl"
layout (location=0) rayPayloadInNV Ray_Payload payload;
layout (location=1) rayPayloadNV Shadow_Ray_Payload shadow_ray_payload;

#include "direct_lighting.glsl"
#include "geometry.glsl"
#include "material_resources.glsl"
#include "compute_bsdf.glsl"
#include "light_resources.glsl"

hitAttributeNV vec2 attribs;

layout(set=KERNEL_SET_0, binding = 1)
uniform accelerationStructureNV accel;

layout(std140, set=KERNEL_SET_0, binding=2)
uniform Uniform_Block {
    mat4x3  camera_to_world;
    int     point_light_count;
    int     diffuse_rectangular_light_count;
    vec2    pad0;
};

Vertex fetch_vertex(int index)
{
    int geometry_buffer_index = instance_infos[gl_InstanceCustomIndexNV].geometry.index; // TODO: do this only if geometry.type is Triangle_Mesh.
    uint vertex_index = index_buffers[geometry_buffer_index].indices[index];
    Mesh_Vertex bv = vertex_buffers[geometry_buffer_index].vertices[vertex_index];

    Vertex v;
    v.p = vec3(bv.x, bv.y, bv.z);
    v.n = vec3(bv.nx, bv.ny, bv.nz);
    v.uv = vec2(bv.u, bv.v);
    return v;
}

Shading_Context init_shading_context()
{
    Vertex v0 = fetch_vertex(gl_PrimitiveID*3 + 0);
    Vertex v1 = fetch_vertex(gl_PrimitiveID*3 + 1);
    Vertex v2 = fetch_vertex(gl_PrimitiveID*3 + 2);

    const vec3 Wo = -gl_WorldRayDirectionNV;
    const mat3 normal_transform = mat3(gl_ObjectToWorldNV);

    // Compute geometric normal.
    // Ensure it's in the same hemisphere as Wo (renderer's convention).
    vec3 Ng = cross(v1.p - v0.p, v2.p - v0.p);
    Ng = normalize(normal_transform * Ng);
    Ng = dot(Ng, Wo) < 0 ? -Ng : Ng;

    // Compute shading normal.
    // Ensure it's in the same hemisphere as Ng (renderer's convention).
    vec3 n0 = normal_transform * v0.n;
    vec3 n1 = normal_transform * v1.n;
    vec3 n2 = normal_transform * v2.n;
    vec3 N = normalize(barycentric_interpolate(attribs.x, attribs.y, n0, n1, n2));
    N = dot(N, Ng) < 0 ? -N : N;

    Shading_Context sc;
    sc.Wo = Wo;
    sc.P = gl_WorldRayOriginNV + gl_WorldRayDirectionNV * gl_HitTNV;
    sc.Ng = Ng;
    sc.N = N;
    sc.UV = barycentric_interpolate(attribs.x, attribs.y, v0.uv, v1.uv, v2.uv);
    return sc;
}

void main()
{
    Shading_Context sc = init_shading_context();
    vec3 L = estimate_direct_lighting(sc, accel, point_light_count, diffuse_rectangular_light_count);

    int area_light_index = instance_infos[gl_InstanceCustomIndexNV].area_light_index; 
    if (area_light_index != -1) {
        L += diffuse_rectangular_lights[area_light_index].emitted_radiance;
    }

    payload.color = srgb_encode(vec3(L));
}
