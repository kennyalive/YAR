#pragma once

#include "../vk.h"

struct Apply_Tone_Mapping {
    VkPipeline pipeline;

    void create(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings);
    void destroy();
    void dispatch(uint32_t output_image_index, uint32_t tonemap_index) const;
};
