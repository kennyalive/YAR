module resources;

import shared;

public struct Mesh_Vertex {
    public float x, y, z;
    public float nx, ny, nz;
    public float u, v;
};

//
// General scene data
//
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

[[vk::binding(5, BASE_SET_INDEX)]]
public StructuredBuffer<Scene_Info> scene_info_buffer;

public Scene_Info get_scene_info()
{
    return scene_info_buffer[0];
}

//
// Material data
//
public static const int Material_Lambertian = 2;
static const int Material_None = -1;

public struct Materials {
    public RWStructuredBuffer<Lambertian_Material> lambertian_materials;
}

//
// Lighting data
//
static const int Light_None = -1;
static const int Light_Point = 0;
static const int Light_Directional = 1;
static const int Light_Rect = 2;

[[vk::binding(POINT_LIGHT_BINDING, LIGHT_SET_INDEX)]]
public StructuredBuffer<Point_Light> point_lights;

[[vk::binding(DIRECTIONAL_LIGHT_BINDING, LIGHT_SET_INDEX)]]
public StructuredBuffer<Directional_Light> directional_lights;

[[vk::binding(RECT_LIGHT_BINDING, LIGHT_SET_INDEX)]]
public StructuredBuffer<Rect_Light> rect_lights;
