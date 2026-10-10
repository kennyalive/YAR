#pragma once

#include "gpu/vk.h"

struct Patch_Materials {
    VkPipeline pipeline;

    void create(const Array<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings);
    void destroy();
    void dispatch(VkCommandBuffer command_buffer);
};
