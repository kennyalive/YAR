#pragma once

#include "../kernels/direct_lighting.h"
#include "../vk.h"

struct Descriptor_Heap;
struct Descriptor_Heap_Layout;
struct GPU_Scene;
struct Kernels;

struct Direct_Lighting_Renderer
{
    Direct_Lighting direct_lighting;

    Vk_Image output_image;
    Vk_Image tonemap;

    Vk_Timer* timer_draw = nullptr;
    Vk_Timer* timer_tonemap = nullptr;
    Vk_Timer* timer_compute_copy = nullptr;

    void initialize(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings, Vk_Time_Keeper& time_keeper);
    void destroy();

    void create_resolution_dependent_resources();
    void destroy_resolution_dependent_resources();
    void write_descriptors(const Descriptor_Heap& descriptor_heap, const Descriptor_Heap_Layout& layout);

    void render(const GPU_Scene& gpu_scene, const Kernels& kernels);
};
