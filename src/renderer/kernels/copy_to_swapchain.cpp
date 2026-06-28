#include "std.h"
#include "lib/common.h"
#include "copy_to_swapchain.h"

#include "renderer/descriptors.h"

void Copy_To_Swapchain::create(const Global_Descriptors& descriptors)
{
    VkDescriptorSetAndBindingMappingEXT mappings[2];
    mappings[0] = map_binding_to_heap_offset(
        0, 0, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT,
        descriptors.tonemapped_image
    );
    mappings[1] = map_binding_to_heap_offset(
        0, 1, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT,
        descriptors.swapchain_images, vk_image_descriptor_size()
    );
    Vk_Shader_Module shader(get_spirv_file("copy_to_swapchain"));
    pipeline = vk_create_compute_pipeline(shader.handle, mappings, "copy_to_swapchain_pipeline");
}

void Copy_To_Swapchain::destroy()
{
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Copy_To_Swapchain::dispatch()
{
    const uint32_t group_size_x = 32; // according to shader
    const uint32_t group_size_y = 32;

    const uint32_t group_count_x = (vk.surface_size.width + group_size_x - 1) / group_size_x;
    const uint32_t group_count_y = (vk.surface_size.height + group_size_y - 1) / group_size_y;

    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdDispatch(vk.command_buffer, group_count_x, group_count_y, 1);
}
