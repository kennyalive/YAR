#pragma once

#include "gpu/vk.h"

struct Scene_Descriptors;

struct Patch_Materials {
    VkPipeline pipeline;

    void create(const Scene_Descriptors& scene_descriptors);
    void destroy();
    void dispatch(VkCommandBuffer command_buffer);
};
