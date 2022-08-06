#pragma once

#include "../vk.h"

struct Apply_Tone_Mapping {
    VkDescriptorSetLayout set_layout;
    VkPipelineLayout pipeline_layout;
    VkPipeline pipeline;
    VkDescriptorSet descriptor_set;

    void create();
    void destroy();
    void update_resolution_dependent_descriptors(VkImageView output_image_view);
    void dispatch();
};
