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
[[vk::binding(SCENE_BINDING_IMAGES, SCENE_SET)]]
public Texture2D images_2d[];

[[vk::binding(SCENE_BINDING_SAMPLER, SCENE_SET)]]
public SamplerState image_sampler;

[[vk::binding(SCENE_BINDING_INSTANCE_INFO, SCENE_SET)]]
public StructuredBuffer<Instance_Info> instance_infos;

[[vk::binding(3, SCENE_SET)]]
public StructuredBuffer<uint> index_buffers[];

[[vk::binding(4, SCENE_SET)]]
public StructuredBuffer<Mesh_Vertex> vertex_buffers[];

[[vk::binding(5, SCENE_SET)]]
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
