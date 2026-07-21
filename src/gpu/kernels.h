#pragma once

#include "kernels/apply_tone_mapping.h"
#include "kernels/patch_materials.h"

struct Descriptor_Offsets;

struct Kernels
{
    Patch_Materials patch_materials;
    Apply_Tone_Mapping apply_tone_mapping;

    void create_kernels(const Descriptor_Offsets& descriptor_offsets);
    void destroy_kernels();
};
