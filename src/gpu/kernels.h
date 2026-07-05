#pragma once

#include "kernels/patch_materials.h"

struct Descriptor_Heap;
struct Global_Descriptors;
struct GPU_Scene;
struct Scene;

struct Kernels
{
    Patch_Materials patch_materials;

    void create_global_kernels(Global_Descriptors& global_descriptors);
    void destroy_global_kernels();

    void create_scene_kernels(Global_Descriptors& global_descriptors, Descriptor_Heap& descriptor_heap, const GPU_Scene& gpu_scene, const Scene& scene);
    void destroy_scene_kernels();
};
