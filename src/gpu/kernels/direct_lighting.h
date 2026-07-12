#pragma once

#include "gpu/vk.h"

struct Direct_Lighting {
    Vk_Buffer shader_binding_table;
    VkPipeline pipeline = VK_NULL_HANDLE;

    void create(
        uint32_t output_image_heap_offset,
        const std::vector<VkDescriptorSetAndBindingMappingEXT>& scene_descriptor_mappings
    );
    void destroy();
    void dispatch();

private:
    void create_pipeline(
        uint32_t output_image_heap_offset,
        const std::vector<VkDescriptorSetAndBindingMappingEXT>& scene_descriptor_mappings
    );
};
