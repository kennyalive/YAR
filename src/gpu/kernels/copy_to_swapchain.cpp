#include "std.h"
#include "lib/common.h"
#include "copy_to_swapchain.h"
#include "../descriptor_heap_layout.h"

#include "shaders/shared.slang"

void Copy_To_Swapchain::create(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings)
{
    Vk_Shader_Module shader(get_spirv_file("copy_to_swapchain"));
    pipeline = vk_create_compute_pipeline(shader.handle, descriptor_mappings, "copy_to_swapchain_pipeline");
}

void Copy_To_Swapchain::destroy()
{
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Copy_To_Swapchain::dispatch(uint32_t tonemap_index) const
{
    const uint32_t group_size_x = 32; // according to shader
    const uint32_t group_size_y = 32;

    const uint32_t group_count_x = (vk.surface_size.width + group_size_x - 1) / group_size_x;
    const uint32_t group_count_y = (vk.surface_size.height + group_size_y - 1) / group_size_y;

    GPU_Types::Copy_To_Swapchain_Params params{};
    params.tonemap_index = tonemap_index;
    params.swapchain_first_image_index = uint32_t(Image_Descriptor_Index::swapchain_first_image);

    VkPushDataInfoEXT push_data_info{ VK_STRUCTURE_TYPE_PUSH_DATA_INFO_EXT };
    push_data_info.offset = sizeof(GPU_Types::Frame_Params);
    push_data_info.data.address = &params;
    push_data_info.data.size = sizeof(params);
    vkCmdPushDataEXT(vk.command_buffer, &push_data_info);

    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdDispatch(vk.command_buffer, group_count_x, group_count_y, 1);
}
