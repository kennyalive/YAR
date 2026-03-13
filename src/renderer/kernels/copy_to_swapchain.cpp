#include "std.h"
#include "lib/common.h"
#include "copy_to_swapchain.h"

#include "../descriptors.h"
#include "../vk_utils.h"

void Copy_To_Swapchain::create(const Descriptors& descriptors)
{
    VkDescriptorSetAndBindingMappingEXT mappings[2];
    mappings[0] = map_binding_to_heap_offset(
        0, 0, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT,
        descriptors.output_image
    );
    mappings[1] = map_binding_to_heap_offset(
        0, 1, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT,
        descriptors.swapchain_images, descriptors.image_descriptor_size
    );
    pipeline = create_compute_pipeline_with_heap_mappings("spirv/copy_to_swapchain.spv", mappings, "copy_to_swapchain_pipeline");
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
