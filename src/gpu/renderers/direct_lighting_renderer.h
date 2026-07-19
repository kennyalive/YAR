#pragma once

#include "../kernels/apply_tone_mapping.h"
#include "../kernels/copy_to_swapchain.h"
#include "../kernels/direct_lighting.h"
#include "../vk.h"

struct Descriptor_Heap;
struct Descriptor_Offsets;
struct GPU_Scene;
struct Scene;

struct Direct_Lighting_Renderer
{
    Apply_Tone_Mapping apply_tone_mapping;
    Copy_To_Swapchain copy_to_swapchain;
    Direct_Lighting direct_lighting;

    Vk_Image output_image;
    Vk_Image tonemapped_image;

    Vk_Timer* timer_draw = nullptr;
    Vk_Timer* timer_tonemap = nullptr;
    Vk_Timer* timer_compute_copy = nullptr;

    void initialize(const Descriptor_Offsets& descriptor_offsets, Vk_Time_Keeper& time_keeper);
    void destroy();

    void create_scene_kernels(const Descriptor_Offsets& descriptor_offsets, const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings);
    void create_resolution_dependent_resources(Descriptor_Heap& descriptor_heap,
        uint32_t output_image_heap_offset, uint32_t tonemap_image_heap_offset, uint32_t swapchain_images_heap_offset);
    void destroy_resolution_dependent_resources();

    void render(const GPU_Scene& gpu_scene);
};
