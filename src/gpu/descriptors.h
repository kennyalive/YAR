#pragma once

struct Descriptor_Heap;

struct Global_Descriptors
{
    uint32_t images = 0;
    uint32_t swapchain_images = 0;
    uint32_t image_sampler = 0;

    void initialize(Descriptor_Heap& descriptor_heap);
};

struct Scene_Descriptors
{
    uint32_t instance_infos = 0;
    uint32_t mesh_infos = 0;
    uint32_t mesh_vertex_data = 0;
    uint32_t mesh_index_data = 0;
    uint32_t scene_info_buffer = 0;
    uint32_t accelerator = 0;
    uint32_t lambertian_materials = 0;
    uint32_t point_lights = 0;
    uint32_t directional_lights = 0;
    uint32_t rect_lights = 0;

    void initialize(Descriptor_Heap& descriptor_heap);
};
