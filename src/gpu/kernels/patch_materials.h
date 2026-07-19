#pragma once

#include "gpu/vk.h"

struct Descriptor_Offsets;

struct Patch_Materials {
    VkPipeline pipeline;

    void create(const Descriptor_Offsets& descriptor_offsets);
    void destroy();
    void dispatch(VkCommandBuffer command_buffer);
};
