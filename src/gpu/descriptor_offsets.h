#pragma once

#include "vk.h"

struct Descriptor_Heap;

constexpr uint32_t max_swapchain_image_descriptors = 4;

enum class Image_Index : uint32_t
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

struct Descriptor_Offsets
{
    uint32_t scene_info_buffer = 0;
    uint32_t instance_infos = 0;
    uint32_t mesh_infos = 0;
    uint32_t mesh_vertex_data = 0;
    uint32_t mesh_index_data = 0;
    uint32_t accelerator = 0;
    uint32_t lambertian_materials = 0;
    uint32_t point_lights = 0;
    uint32_t directional_lights = 0;
    uint32_t rect_lights = 0;

    // Images is the only array of descriptors, so it is in the end.
    // This allows to have predefined offsets for all descriptors.
    uint32_t images = 0;

    // Sampler heap
    uint32_t image_sampler = 0;

    void initialize(Descriptor_Heap& descriptor_heap);
    uint32_t get_image_descriptor_offset(Image_Index image_type) const;
    std::vector<VkDescriptorSetAndBindingMappingEXT> get_descriptor_mappings() const;
};
