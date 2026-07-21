module resources;

import shared;

public struct Mesh_Vertex {
    public float x, y, z;
    public float nx, ny, nz;
    public float u, v;
};

//
// Global image array.
// All image arrays alias the same descriptor array
//
[[vk::binding(GLOBAL_BINDING_SAMPLED_IMAGES, GLOBAL_SET)]]
public Texture2D<float4> sampled_images[];

[[vk::image_format("rgba16f")]]
[[vk::binding(GLOBAL_BINDING_STORAGE_IMAGES_RGBA_F16, GLOBAL_SET)]]
public RWTexture2D<float4> storage_images_rgba_f16[];

[[vk::binding(GLOBAL_BINDING_SAMPLER, GLOBAL_SET)]]
public SamplerState image_sampler;

//
// General scene data
//
[[vk::binding(SCENE_BASE_BINDING_INSTANCE_INFO, SCENE_BASE_SET)]]
public StructuredBuffer<Instance_Info> instance_infos;

[[vk::binding(SCENE_BASE_BINDING_MESH_INFO, SCENE_BASE_SET)]]
public StructuredBuffer<Mesh_Info> mesh_infos;

[[vk::binding(SCENE_BASE_BINDING_MESH_VERTEX_DATA, SCENE_BASE_SET)]]
public StructuredBuffer<Mesh_Vertex> mesh_vertex_data;

[[vk::binding(SCENE_BASE_BINDING_MESH_INDEX_DATA, SCENE_BASE_SET)]]
public StructuredBuffer<uint> mesh_index_data;

[[vk::binding(SCENE_BASE_BINDING_SCENE_INFO, SCENE_BASE_SET)]]
public StructuredBuffer<Scene_Info> scene_info_buffer;

[[vk::binding(SCENE_BASE_BINDING_ACCELERATOR, SCENE_BASE_SET)]]
public RaytracingAccelerationStructure accel;

public Scene_Info get_scene_info()
{
    return scene_info_buffer[0];
}

//
// Material data
//
public static const int Material_Lambertian = 2;
static const int Material_None = -1;

[[vk::binding(SCENE_MATERIAL_BINDING_LAMBERTIAN, SCENE_MATERIAL_SET)]]
public RWStructuredBuffer<Lambertian_Material> lambertian_materials;

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
