module resources;

import shared;

public struct Mesh_Vertex {
    public float x, y, z;
    public float nx, ny, nz;
    public float u, v;
};

//
// Global renderer data
//
[[vk::binding(GLOBAL_BINDING_IMAGES, GLOBAL_SET)]]
public Texture2D images[];

[[vk::binding(GLOBAL_BINDING_SAMPLER, GLOBAL_SET)]]
public SamplerState image_sampler;

//
// General scene data
//
[[vk::binding(SCENE_BASE_BINDING_INSTANCE_INFO, SCENE_BASE_SET)]]
public StructuredBuffer<Instance_Info> instance_infos;

[[vk::binding(SCENE_BASE_BINDING_INDEX_BUFFERS, SCENE_BASE_SET)]]
public StructuredBuffer<uint> index_buffers[];

[[vk::binding(SCENE_BASE_BINDING_VERTEX_BUFFERS, SCENE_BASE_SET)]]
public StructuredBuffer<Mesh_Vertex> vertex_buffers[];

[[vk::binding(SCENE_BASE_BINDING_SCENE_INFO, SCENE_BASE_SET)]]
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

[[vk::binding(SCENE_LIGHT_BINDING_POINT, SCENE_LIGHT_SET)]]
public StructuredBuffer<Point_Light> point_lights;

[[vk::binding(SCENE_LIGHT_BINDING_DIRECTIONAL, SCENE_LIGHT_SET)]]
public StructuredBuffer<Directional_Light> directional_lights;

[[vk::binding(SCENE_LIGHT_BINDING_RECT, SCENE_LIGHT_SET)]]
public StructuredBuffer<Rect_Light> rect_lights;
