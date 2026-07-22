#pragma once

#include "../kernels/direct_lighting.h"
#include "../vk.h"

struct Descriptor_Heap;
struct GPU_Scene;
struct Kernels;

struct Direct_Lighting_Renderer
{
    Direct_Lighting direct_lighting;

    Vk_Image output_image;
    Vk_Image tonemapped_image;

    Vk_Timer* timer_draw = nullptr;
    Vk_Timer* timer_tonemap = nullptr;
    Vk_Timer* timer_compute_copy = nullptr;

    void initialize(Vk_Time_Keeper& time_keeper);
    void destroy();

    void create_kernels(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings);
    void create_resolution_dependent_resources(Descriptor_Heap& descriptor_heap,
        uint32_t output_image_heap_offset, uint32_t tonemap_image_heap_offset);
    void destroy_resolution_dependent_resources();

    void render(const GPU_Scene& gpu_scene, const Kernels& kernels);
};
