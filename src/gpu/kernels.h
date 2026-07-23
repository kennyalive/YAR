#pragma once

#include "kernels/apply_tone_mapping.h"
#include "kernels/copy_to_swapchain.h"
#include "kernels/patch_materials.h"

struct Descriptor_Heap_Layout;

struct Kernels
{
    Patch_Materials patch_materials;
    Apply_Tone_Mapping apply_tone_mapping;
    Copy_To_Swapchain copy_to_swapchain;

    void create_kernels(const Descriptor_Heap_Layout& layout);
    void destroy_kernels();
};
