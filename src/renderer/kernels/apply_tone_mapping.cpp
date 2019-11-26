#include "std.h"
#include "apply_tone_mapping.h"

#include "../utils.h"

void Apply_Tone_Mapping::create() {
    set_layout = Descriptor_Set_Layout()
        .sampler(0, VK_SHADER_STAGE_COMPUTE_BIT)
        .sampled_image(1, VK_SHADER_STAGE_COMPUTE_BIT)
        .storage_image(2, VK_SHADER_STAGE_COMPUTE_BIT)
        .create("apply_tone_mapping_set_layout");

    pipeline_layout = create_pipeline_layout(
        { set_layout },
        { VkPushConstantRange{VK_SHADER_STAGE_COMPUTE_BIT, 0, 8 /*uint32 width + uint32 height*/} },
        "apply_tone_mapping_pipeline_layout");

    pipeline = create_compute_pipeline("spirv/apply_tone_mapping.comp.spv", pipeline_layout, "apply_tone_mapping_pipeline_layout");

    // point sampler
    {
        VkSamplerCreateInfo create_info { VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO };
        VK_CHECK(vkCreateSampler(vk.device, &create_info, nullptr, &point_sampler));
        vk_set_debug_name(point_sampler, "point_sampler");
    }

    descriptor_set = allocate_descriptor_set(set_layout);
    Descriptor_Writes(descriptor_set).sampler(0, point_sampler);
}

void Apply_Tone_Mapping::destroy() {
    vkDestroyDescriptorSetLayout(vk.device, set_layout, nullptr);
    vkDestroyPipelineLayout(vk.device, pipeline_layout, nullptr);
    vkDestroyPipeline(vk.device, pipeline, nullptr);
    vkDestroySampler(vk.device, point_sampler, nullptr);
}

void Apply_Tone_Mapping::update_resolution_dependent_descriptors(VkImageView output_image_view) {
    Descriptor_Writes(descriptor_set)
        .sampled_image(1, output_image_view, VK_IMAGE_LAYOUT_GENERAL)
        .storage_image(2, output_image_view);
}

void Apply_Tone_Mapping::dispatch() {
    const uint32_t group_size_x = 32; // according to shader
    const uint32_t group_size_y = 32;

    uint32_t group_count_x = (vk.surface_size.width + group_size_x - 1) / group_size_x;
    uint32_t group_count_y = (vk.surface_size.height + group_size_y - 1) / group_size_y;
    uint32_t push_constants[] = { vk.surface_size.width, vk.surface_size.height };

    vkCmdPushConstants(vk.command_buffer, pipeline_layout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(push_constants), push_constants);
    vkCmdBindDescriptorSets(vk.command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout, 0, 1, &descriptor_set, 0, nullptr);
    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdDispatch(vk.command_buffer, group_count_x, group_count_y, 1);
}
