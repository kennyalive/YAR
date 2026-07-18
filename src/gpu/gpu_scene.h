#pragma once

#include "acceleration_structure.h"
#include "descriptors.h"
#include "lib/material.h"
#include "vk.h"

struct Descriptor_Heap;
struct Global_Descriptors;
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
    Scene_Descriptors descriptors;

    std::vector<Vk_Image> images;
    std::vector<GPU_Mesh> meshes;
    Vk_Intersection_Accelerator accelerator;
    Vk_Buffer mesh_vertex_data;
    Vk_Buffer mesh_index_data;
    Vk_Buffer instance_info_buffer;
    Vk_Buffer scene_info_buffer;
    Vk_Buffer point_lights;
    Vk_Buffer directional_lights;
    Vk_Buffer rect_lights;
    Vk_Buffer lambertian_material_buffer;

    void load(const Scene& scene, Descriptor_Heap& descriptor_heap, Global_Descriptors& global_descriptors);
    void destroy();
    std::vector<VkDescriptorSetAndBindingMappingEXT> get_scene_descriptor_mappings() const;

private:
    void write_descriptors(Descriptor_Heap& descriptor_heap, Global_Descriptors& global_descriptors);
};
