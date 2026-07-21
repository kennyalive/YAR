#pragma once

#include "gpu/vk.h"

struct Direct_Lighting
{
    Vk_Buffer shader_binding_table;
    VkPipeline pipeline = VK_NULL_HANDLE;

    void create(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings);
    void destroy();
    void dispatch(uint32_t output_image_index);

private:
    void create_pipeline(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings);
};
