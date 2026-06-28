#pragma once

#include "kernels/apply_tone_mapping.h"
#include "kernels/copy_to_swapchain.h"
#include "kernels/patch_materials.h"
#include "kernels/direct_lighting.h"
#include "kernels/path_tracing.h"

struct Descriptor_Heap;
struct Global_Descriptors;
struct GPU_Scene;
struct Scene;

struct Kernels
{
    Apply_Tone_Mapping apply_tone_mapping;
    Copy_To_Swapchain copy_to_swapchain;

    Patch_Materials patch_materials;
    Direct_Lighting direct_lighting;
    Path_Tracing path_tracing;

    void create_global_kernels(Global_Descriptors& global_descriptors);
    void create_scene_kernels(Global_Descriptors& global_descriptors, Descriptor_Heap& descriptor_heap, const GPU_Scene& gpu_scene, const Scene& scene);
    void destroy_global_kernels();
    void destroy_scene_kernels();
};
