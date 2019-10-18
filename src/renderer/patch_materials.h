#pragma once

#include "vk.h"

struct Patch_Materials {
    VkPipelineLayout pipeline_layout;
    VkPipeline pipeline;

    void create(VkDescriptorSetLayout material_descriptor_set_layout);
    void destroy();
    void dispatch(VkCommandBuffer command_buffer, VkDescriptorSet material_descriptor_set);
};

