#include "std.h"
#include "lib/common.h"
#include "apply_tone_mapping.h"

#include "../descriptors.h"
#include "../vk_utils.h"

void Apply_Tone_Mapping::create(Descriptors&  descriptors)
{
    VkDescriptorMappingSourceConstantOffsetEXT constant_offset{};
    constant_offset.heapOffset = descriptors.output_image;

    VkDescriptorSetAndBindingMappingEXT mapping{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_AND_BINDING_MAPPING_EXT };
    mapping.descriptorSet = 0;
    mapping.firstBinding = 0;
    mapping.bindingCount = 1;
    mapping.resourceMask = VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT;
    mapping.source = VK_DESCRIPTOR_MAPPING_SOURCE_HEAP_WITH_CONSTANT_OFFSET_EXT;
    mapping.sourceData.constantOffset = constant_offset;

    VkShaderDescriptorSetAndBindingMappingInfoEXT mapping_info{ VK_STRUCTURE_TYPE_SHADER_DESCRIPTOR_SET_AND_BINDING_MAPPING_INFO_EXT };
    mapping_info.mappingCount = 1;
    mapping_info.pMappings = &mapping;

    pipeline = create_compute_pipeline_with_heap_mappings("spirv/apply_tone_mapping.spv", mapping_info, "apply_tone_mapping_pipeline_layout");
}

void Apply_Tone_Mapping::destroy() {
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Apply_Tone_Mapping::dispatch()
{
    const uint32_t group_size_x = 32; // according to shader
    const uint32_t group_size_y = 32;

    uint32_t group_count_x = (vk.surface_size.width + group_size_x - 1) / group_size_x;
    uint32_t group_count_y = (vk.surface_size.height + group_size_y - 1) / group_size_y;
    uint32_t push_data[] = { vk.surface_size.width / 2, vk.surface_size.height };

    VkPushDataInfoEXT push_data_info{ VK_STRUCTURE_TYPE_PUSH_DATA_INFO_EXT };
    push_data_info.data.address = push_data;
    push_data_info.data.size = 8;
    vkCmdPushDataEXT(vk.command_buffer, &push_data_info);

    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdDispatch(vk.command_buffer, group_count_x, group_count_y, 1);
}
