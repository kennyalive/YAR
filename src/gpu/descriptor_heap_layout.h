#pragma once

#include "vk.h"

constexpr uint32_t max_swapchain_image_descriptors = 4;

enum class Image_Descriptor_Index : uint32_t
{
    none,
    black,
    swapchain_first_image,
    swapchain_last_image = swapchain_first_image + max_swapchain_image_descriptors - 1,
    path_tracer_output,
    path_tracer_tonemap,
    direct_lighting_output,
    direct_lighting_tonemap,

    // Project image indices use a separate numeric range for easier identification.
    first_project_image = 100
};

struct Descriptor_Heap_Layout
{
    // Scene
    uint32_t scene_info = 0;
    uint32_t instance_infos = 0;
    uint32_t mesh_infos = 0;
    uint32_t mesh_vertex_data = 0;
    uint32_t mesh_index_data = 0;

    uint32_t accelerator = 0;

    // Materials
    uint32_t lambertian_materials = 0;

    // Lights
    uint32_t point_lights = 0;
    uint32_t directional_lights = 0;
    uint32_t rect_lights = 0;

    // Array of image descriptors. It is the only array in entire heap layout.
    // Having single only single array at the end of the heap makes descriptor
    // offsets independent of the loaded project.
    uint32_t images = 0;

    // Sampler heap
    uint32_t image_sampler = 0;

    // All of the above offsets that are buffer descriptor offsets.
    // This is useful during cleanup, when we want to zero all buffer descriptors.
    std::vector<uint32_t> storage_buffer_descriptor_offsets;

    void initialize();
    uint32_t get_image_descriptor_offset(Image_Descriptor_Index index) const;
    uint32_t get_total_descriptor_data_size(uint32_t project_image_count) const;
    std::vector<VkDescriptorSetAndBindingMappingEXT> get_descriptor_mappings() const;
};
