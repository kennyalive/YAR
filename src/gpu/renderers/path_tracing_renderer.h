#pragma once

#include "../kernels/apply_tone_mapping.h"
#include "../kernels/copy_to_swapchain.h"
#include "../kernels/path_tracing.h"
#include "../vk.h"

struct Descriptor_Heap;
struct GPU_Scene;
struct Scene;

struct Path_Tracing_Renderer
{
    Apply_Tone_Mapping apply_tone_mapping;
    Copy_To_Swapchain copy_to_swapchain;
    Path_Tracing path_tracing;

    Vk_Image output_image;
    uint32_t output_image_heap_offset = -1;

    Vk_Image tonemapped_image;
    uint32_t tonemapped_image_heap_offset = -1;

    Vk_Timer* timer_draw = nullptr;
    Vk_Timer* timer_tonemap = nullptr;
    Vk_Timer* timer_compute_copy = nullptr;

    void initialize(Descriptor_Heap& descriptor_heap, Vk_Time_Keeper& time_keeper);
    void destroy();

    void create_scene_kernels(Descriptor_Heap& descriptor_heap, const GPU_Scene& gpu_scene, const Scene& scene);
    void create_resolution_dependent_resources(Descriptor_Heap& descriptor_heap, uint32_t swapchain_images_heap_offset);
    void destroy_resolution_dependent_resources();

    void render(const GPU_Scene& gpu_scene);
};
