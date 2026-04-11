import common;
import shared_main;

import direct_lighting;
import raytrace_types;
import shading_context;

import base_resources;
import light_resources;
import material_resources;

[[vk::binding(0, 1)]]
ParameterBlock<Materials> materials;

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

float solve_linear_system_2x2(float a[2][2], float3 b[2], out float3 x1, out float3 x2)
{
    float determinant = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (determinant != 0.f) {
        float inv_det = 1.f / determinant;
        x1 = (a[1][1] * b[0] - a[0][1] * b[1]) * inv_det;
        x2 = (-a[1][0] * b[0] + a[0][0] * b[1]) * inv_det;
    }
    return determinant;
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

    float3 n;
    // If there is no normal then use a geometry normal. 
    // TODO: support separate buffer per attribute and also have
    // boolean constant/flag to check if normals are available.
    if (any(v0.n != float3(0, 0, 0))) {
        // Compute shading normal.
        // Ensure it's in the same hemisphere as Ng (renderer's convention).
        float3 n0 = mul(normal_transform, v0.n);
        float3 n1 = mul(normal_transform, v1.n);
        float3 n2 = mul(normal_transform, v2.n);
        n = normalize(barycentric_interpolate(attribs.x, attribs.y, n0, n1, n2));
        n = dot(n, ng) < 0 ? -n : n;
    }
    else {
        n = ng;
    }

    // Compute dpdu/dpdv
    float3 dpdu, dpdv;
    {
        float a[2][2] = {
            { v1.uv.x - v0.uv.x, v1.uv.y - v0.uv.y },
            { v2.uv.x - v0.uv.x, v2.uv.y - v0.uv.y }
        };
        float3 bp[2] = { v1.p - v0.p, v2.p - v0.p };
        float det = solve_linear_system_2x2(a, bp, dpdu, dpdv);
    }

    Shading_Context sc;
    sc.Wo = wo;
    sc.P = WorldRayOrigin() + WorldRayDirection() * RayTCurrent();
    sc.Ng = ng;
    sc.N = n;
    sc.UV = barycentric_interpolate(attribs.x, attribs.y, v0.uv, v1.uv, v2.uv);
    sc.dpdu = dpdu;
    sc.dpdv = dpdv;
    return sc;
}

[shader("closesthit")]
void main(inout Path_Payload payload, in Attribs attribs)
{
    Shading_Context sc = init_shading_context(attribs.barycentrics);
    
    float3 L = estimate_direct_lighting(materials, sc, accel, point_light_count, directional_light_count, diffuse_rectangular_light_count);

    int area_light_index = instance_infos[InstanceID()].area_light_index;
    if (area_light_index != -1) {
        L += diffuse_rectangular_lights[area_light_index].emitted_radiance;
    }

    payload.hit = true;
    payload.color = srgb_encode(L);
    payload.t = RayTCurrent();
    payload.normal = sc.N;
    payload.dpdu = sc.dpdu;
    payload.uv = sc.UV;
    payload.mtl_handle = instance_infos[InstanceID()].material;
}
