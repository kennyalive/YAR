#pragma once

#include "renderer/vk.h"

struct Descriptors;

struct Patch_Materials {
    VkPipeline pipeline;

    void create(Descriptors& descriptors);
    void destroy();
    void dispatch(VkCommandBuffer command_buffer);
};
