#include "std.h"
#include "lib/common.h"
#include "copy_to_swapchain.h"

#include "../descriptors.h"
#include "../vk_utils.h"

void Copy_To_Swapchain::create(const Descriptors& descriptors)
{
    VkDescriptorMappingSourceConstantOffsetEXT constant_offset0{};
    constant_offset0.heapOffset = descriptors.output_image;

    VkDescriptorSetAndBindingMappingEXT mapping0{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_AND_BINDING_MAPPING_EXT };
    mapping0.descriptorSet = 0;
    mapping0.firstBinding = 0;
    mapping0.bindingCount = 1;
    mapping0.resourceMask = VK_SPIRV_RESOURCE_TYPE_SAMPLED_IMAGE_BIT_EXT;
    mapping0.source = VK_DESCRIPTOR_MAPPING_SOURCE_HEAP_WITH_CONSTANT_OFFSET_EXT;
    mapping0.sourceData.constantOffset = constant_offset0;

    VkDescriptorMappingSourceConstantOffsetEXT constant_offset1{};
    constant_offset1.heapOffset = descriptors.swapchain_images;
    constant_offset1.heapArrayStride = descriptors.image_descriptor_size;

    VkDescriptorSetAndBindingMappingEXT mapping1{ VK_STRUCTURE_TYPE_DESCRIPTOR_SET_AND_BINDING_MAPPING_EXT };
    mapping1.descriptorSet = 0;
    mapping1.firstBinding = 1;
    mapping1.bindingCount = 1;
    mapping1.resourceMask = VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT;
    mapping1.source = VK_DESCRIPTOR_MAPPING_SOURCE_HEAP_WITH_CONSTANT_OFFSET_EXT;
    mapping1.sourceData.constantOffset = constant_offset1;

    VkDescriptorSetAndBindingMappingEXT mappings[2] = { mapping0, mapping1 };

    VkShaderDescriptorSetAndBindingMappingInfoEXT mapping_info{ VK_STRUCTURE_TYPE_SHADER_DESCRIPTOR_SET_AND_BINDING_MAPPING_INFO_EXT };
    mapping_info.mappingCount = 2;
    mapping_info.pMappings = mappings;

    pipeline = create_compute_pipeline_with_heap_mappings("spirv/copy_to_swapchain.spv", mapping_info, "copy_to_swapchain_pipeline");
}

void Copy_To_Swapchain::destroy()
{
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Copy_To_Swapchain::dispatch()
{
    const uint32_t group_size_x = 32; // according to shader
    const uint32_t group_size_y = 32;

    uint32_t group_count_x = (vk.surface_size.width + group_size_x - 1) / group_size_x;
    uint32_t group_count_y = (vk.surface_size.height + group_size_y - 1) / group_size_y;
    uint32_t push_data[] = { vk.surface_size.width, vk.surface_size.height, vk.swapchain_image_index };

    VkPushDataInfoEXT push_data_info{ VK_STRUCTURE_TYPE_PUSH_DATA_INFO_EXT };
    push_data_info.data.address = push_data;
    push_data_info.data.size = 12;
    vkCmdPushDataEXT(vk.command_buffer, &push_data_info);

    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdDispatch(vk.command_buffer, group_count_x, group_count_y, 1);
}
