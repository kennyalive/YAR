#include "std.h"
#include "lib/common.h"
#include "patch_materials.h"
#include "renderer/utils.h"

void Patch_Materials::create(VkDescriptorSetLayout material_descriptor_set_layout) {
    {
        VkPipelineLayoutCreateInfo create_info{ VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        create_info.setLayoutCount = 1;
        create_info.pSetLayouts = &material_descriptor_set_layout;
        VK_CHECK(vkCreatePipelineLayout(vk.device, &create_info, nullptr, &pipeline_layout));
    }
    {
        Shader_Module shader("spirv/patch_materials.comp.spv");

        VkPipelineShaderStageCreateInfo compute_stage{ VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO };
        compute_stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        compute_stage.module = shader.handle;
        compute_stage.pName = "main";

        VkComputePipelineCreateInfo create_info{ VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO };
        create_info.stage = compute_stage;
        create_info.layout = pipeline_layout;
        VK_CHECK(vkCreateComputePipelines(vk.device, VK_NULL_HANDLE, 1, &create_info, nullptr, &pipeline));
    }
}

void Patch_Materials::destroy() {
    vkDestroyPipelineLayout(vk.device, pipeline_layout, nullptr);
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Patch_Materials::dispatch(VkCommandBuffer command_buffer, VkDescriptorSet material_descriptor_set) {
    vkCmdBindDescriptorSets(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout, 0,
        1, &material_descriptor_set, 0, nullptr);
    vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdDispatch(command_buffer, 1, 1, 1);
}

