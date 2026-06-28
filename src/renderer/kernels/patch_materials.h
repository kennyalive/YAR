#pragma once

#include "renderer/vk.h"

struct Scene_Descriptors;

struct Patch_Materials {
    VkPipeline pipeline;

    void create(Scene_Descriptors& scene_descriptors);
    void destroy();
    void dispatch(VkCommandBuffer command_buffer);
};
