#pragma once

#include "acceleration_structure.h"
#include "lib/material.h"

struct Descriptor_Heap;
struct Descriptor_Heap_Layout;
struct Scene;

struct GPU_Mesh 
{
    uint64_t first_vertex_offset = -1;
    uint64_t first_index_offset = -1;
    uint32_t vertex_count = 0;
    uint32_t index_count = 0;
    Material_Handle material;
    int area_light_index = -1;
};

struct GPU_Scene
{
    bool loaded = false;

    std::vector<Vk_Image> images;
    std::vector<GPU_Mesh> meshes;
    Vk_Intersection_Accelerator accelerator;

    Vk_Buffer scene_info;
    Vk_Buffer instance_infos;
    Vk_Buffer mesh_infos;
    Vk_Buffer mesh_vertex_data;
    Vk_Buffer mesh_index_data;
    Vk_Buffer lambertian_materials;
    Vk_Buffer point_lights;
    Vk_Buffer directional_lights;
    Vk_Buffer rect_lights;

    void load(const Scene& scene);
    void destroy();
    void write_descriptors(Descriptor_Heap& descriptor_heap, const Descriptor_Heap_Layout& layout);
    void clear_descriptors(Descriptor_Heap& descriptor_heap, const Descriptor_Heap_Layout& layout);
};
