#pragma once

#include "gpu/vk.h"

struct Path_Tracing
{
    Vk_Buffer shader_binding_table;
    VkPipeline pipeline = VK_NULL_HANDLE;

    void create(const Array<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings);
    void destroy();
    void dispatch(uint32_t output_image_index, uint32_t accumulation_index);

private:
    void create_pipeline(const Array<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings);
};
