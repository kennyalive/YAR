#include "common.hlsli"
#include "base_resources.hlsli"
#include "direct_lighting.hlsli"
#include "light_resources.hlsli"
#include "rt_utils.hlsli"
#include "shading_context.hlsli"

[[vk::binding(1, KERNEL_SET_0)]]
RaytracingAccelerationStructure accel;

[[vk::binding(2, KERNEL_SET_0)]]
cbuffer Constants {
    float3x4 camera_to_world;
    int point_light_count;
    int directional_light_count;
    int diffuse_rectangular_light_count;
    float2 pad0;
};

Vertex fetch_vertex(int index)
{
    // TODO: here we assume that geometry.type is Triangle_Mesh. Will be invalid in the presence of other geom types.
    int geometry_buffer_index = instance_infos[InstanceID()].geometry.index;

    uint vertex_index = index_buffers[geometry_buffer_index][index];
    Mesh_Vertex bv = vertex_buffers[geometry_buffer_index][vertex_index];

    Vertex v;
    v.p = float3(bv.x, bv.y, bv.z);
    v.n = float3(bv.nx, bv.ny, bv.nz);
    v.uv = float2(bv.u, bv.v);
    return v;
}

Shading_Context init_shading_context(float2 attribs)
{
    Vertex v0 = fetch_vertex(PrimitiveIndex() * 3 + 0);
    Vertex v1 = fetch_vertex(PrimitiveIndex() * 3 + 1);
    Vertex v2 = fetch_vertex(PrimitiveIndex() * 3 + 2);

    float3 wo = -WorldRayDirection();
    float3x3 normal_transform = (float3x3) ObjectToWorld3x4();

    // Compute geometric normal.
    // Ensure it's in the same hemisphere as Wo (renderer's convention).
    float3 ng = cross(v1.p - v0.p, v2.p - v0.p);
    ng = normalize(mul(normal_transform, ng));
    ng = dot(ng, wo) < 0 ? -ng : ng;

    // Compute shading normal.
    // Ensure it's in the same hemisphere as Ng (renderer's convention).
    float3 n0 = mul(normal_transform, v0.n);
    float3 n1 = mul(normal_transform, v1.n);
    float3 n2 = mul(normal_transform, v2.n);
    float3 n = normalize(barycentric_interpolate(attribs.x, attribs.y, n0, n1, n2));
    n = dot(n, ng) < 0 ? -n : n;

    Shading_Context sc;
    sc.Wo = wo;
    sc.P = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();
    sc.Ng = ng;
    sc.N = n;
    sc.UV = barycentric_interpolate(attribs.x, attribs.y, v0.uv, v1.uv, v2.uv);
    return sc;
}

[shader("closesthit")]
void main(inout Ray_Payload payload, in Attribs attribs)
{
    Shading_Context sc = init_shading_context(attribs.barycentrics);
    
    float3 L = estimate_direct_lighting(sc, accel, point_light_count, directional_light_count, diffuse_rectangular_light_count);

    int area_light_index = instance_infos[InstanceID()].area_light_index;
    if (area_light_index != -1) {
        L += diffuse_rectangular_lights[area_light_index].emitted_radiance;
    }

    payload.color = srgb_encode(L);
}
