#include "std.h"
#include "lib/common.h"
#include "apply_tone_mapping.h"

void Apply_Tone_Mapping::create(uint32_t output_image_heap_offset, uint32_t tonemapped_image_heap_offset)
{
    const VkDescriptorSetAndBindingMappingEXT output_image_mapping = map_binding_to_heap_offset(
        0, 0, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT, output_image_heap_offset
    );
    const VkDescriptorSetAndBindingMappingEXT tonemapped_image_mapping = map_binding_to_heap_offset(
        0, 1, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT, tonemapped_image_heap_offset
    );
    const VkDescriptorSetAndBindingMappingEXT mappings[2] = {
            output_image_mapping,
            tonemapped_image_mapping,
    };
    Vk_Shader_Module shader(get_spirv_file("apply_tone_mapping"));
    pipeline = vk_create_compute_pipeline(shader.handle, std::span(mappings, 2), "apply_tone_mapping_pipeline_layout");
}

void Apply_Tone_Mapping::destroy()
{
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Apply_Tone_Mapping::dispatch()
{
    const uint32_t group_size_x = 32; // according to shader
    const uint32_t group_size_y = 32;

    const uint32_t group_count_x = (vk.surface_size.width + group_size_x - 1) / group_size_x;
    const uint32_t group_count_y = (vk.surface_size.height + group_size_y - 1) / group_size_y;

    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdDispatch(vk.command_buffer, group_count_x, group_count_y, 1);
}
