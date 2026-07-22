#pragma once

#include "../vk.h"

struct Copy_To_Swapchain {
    VkPipeline pipeline;

    void create(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings);
    void destroy();
    void dispatch(uint32_t tonemap_index) const;
};
