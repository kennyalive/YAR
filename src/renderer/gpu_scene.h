#pragma once

#include "descriptors.h"
#include "vk.h"
#include "lib/material.h"

struct Descriptor_Heap;
struct Scene;

struct GPU_Mesh 
{
    Vk_Buffer vertex_buffer;
    Vk_Buffer index_buffer;
    uint32_t vertex_count = 0;
    uint32_t index_count = 0;
    Material_Handle material;
    int area_light_index = -1;
};

struct GPU_Scene
{
    bool loaded = false;
    Scene_Descriptors descriptors;

    std::vector<Vk_Image> images_2d;
    std::vector<GPU_Mesh> meshes;
    Vk_Buffer instance_info_buffer;
    Vk_Buffer scene_info_buffer;
    Vk_Buffer point_lights;
    Vk_Buffer directional_lights;
    Vk_Buffer rect_lights;
    Vk_Buffer lambertian_material_buffer;

    void load(const Scene& scene, Descriptor_Heap& descriptor_heap);
    void destroy();
    std::vector<VkDescriptorSetAndBindingMappingEXT> get_scene_descriptor_mappings() const;

private:
    void write_descriptors(Descriptor_Heap& descriptor_heap);
};
