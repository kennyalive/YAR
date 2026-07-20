#pragma once

#include "kernels/apply_tone_mapping.h"
#include "kernels/patch_materials.h"

struct Descriptor_Heap;
struct Descriptor_Offsets;
struct GPU_Scene;
struct Scene;

struct Kernels
{
    Patch_Materials patch_materials;
    Apply_Tone_Mapping apply_tone_mapping;

    void create_global_kernels(Descriptor_Offsets& descriptor_offsets);
    void destroy_global_kernels();

    void create_scene_kernels(Descriptor_Offsets& descriptor_offsets, Descriptor_Heap& descriptor_heap, const GPU_Scene& gpu_scene, const Scene& scene);
    void destroy_scene_kernels();
};
