module base_resources;

import shared_main;

public struct Mesh_Vertex {
    public float x, y, z;
    public float nx, ny, nz;
    public float u, v;
};

[[vk::binding(0, BASE_SET_INDEX)]]
public Texture2D images_2d[];

[[vk::binding(1, BASE_SET_INDEX)]]
public SamplerState image_sampler;

[[vk::binding(BASE_SET_BINDING_INSTANCE_INFO, BASE_SET_INDEX)]]
public StructuredBuffer<Instance_Info> instance_infos;

[[vk::binding(3, BASE_SET_INDEX)]]
public StructuredBuffer<uint> index_buffers[];

[[vk::binding(4, BASE_SET_INDEX)]]
public StructuredBuffer<Mesh_Vertex> vertex_buffers[];
