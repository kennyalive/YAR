#include "std.h"
#include "lib/common.h"
#include "apply_tone_mapping.h"

#include "shaders/shared.slang"

void Apply_Tone_Mapping::create(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings)
{
    Vk_Shader_Module shader(get_spirv_file("apply_tone_mapping"));
    pipeline = vk_create_compute_pipeline(shader.handle, descriptor_mappings, "apply_tone_mapping_pipeline_layout");
}

void Apply_Tone_Mapping::destroy()
{
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Apply_Tone_Mapping::dispatch(uint32_t output_image_index, uint32_t tonemap_index)
{
    const uint32_t group_size_x = 32; // according to shader
    const uint32_t group_size_y = 32;

    const uint32_t group_count_x = (vk.surface_size.width + group_size_x - 1) / group_size_x;
    const uint32_t group_count_y = (vk.surface_size.height + group_size_y - 1) / group_size_y;

    GPU_Types::Apply_Tonemap_Params params{};
    params.output_image_index = output_image_index;
    params.tonemap_index = tonemap_index;

    VkPushDataInfoEXT push_data_info{ VK_STRUCTURE_TYPE_PUSH_DATA_INFO_EXT };
    push_data_info.offset = sizeof(GPU_Types::Frame_Params);
    push_data_info.data.address = &params;
    push_data_info.data.size = sizeof(GPU_Types::Apply_Tonemap_Params);
    vkCmdPushDataEXT(vk.command_buffer, &push_data_info);

    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdDispatch(vk.command_buffer, group_count_x, group_count_y, 1);
}
