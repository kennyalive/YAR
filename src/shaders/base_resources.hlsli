#ifndef BASE_RESOURCES_HLSL
#define BASE_RESOURCES_HLSL

#include "shared_main.hlsli"

[[vk::binding(0, BASE_SET_INDEX)]]
Texture2D images_2d[];

[[vk::binding(1, BASE_SET_INDEX)]]
SamplerState image_sampler;

[[vk::binding(BASE_SET_BINDING_INSTANCE_INFO, BASE_SET_INDEX)]]
StructuredBuffer<Instance_Info> instance_infos;

[[vk::binding(3, BASE_SET_INDEX)]]
StructuredBuffer<uint> index_buffers[];

struct Mesh_Vertex {
    float x, y, z;
    float nx, ny, nz;
    float u, v;
};

[[vk::binding(4, BASE_SET_INDEX)]]
StructuredBuffer<Mesh_Vertex> vertex_buffers[];

#endif
