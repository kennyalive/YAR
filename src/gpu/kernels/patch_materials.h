#pragma once

#include "gpu/vk.h"

struct Patch_Materials {
    VkPipeline pipeline;

    void create(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings);
    void destroy();
    void dispatch(VkCommandBuffer command_buffer);
};
