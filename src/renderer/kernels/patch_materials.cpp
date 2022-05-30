#include "std.h"
#include "lib/common.h"
#include "patch_materials.h"
#include "renderer/vk_utils.h"

void Patch_Materials::create(VkDescriptorSetLayout material_descriptor_set_layout) {
    pipeline_layout = create_pipeline_layout(
        { material_descriptor_set_layout },
        {},
        "patch_materials_pipeline_layout"
    );

    pipeline = create_compute_pipeline("spirv/patch_materials.comp.spv", pipeline_layout,
        "patch_materials_pipeline");
}

void Patch_Materials::destroy() {
    vkDestroyPipelineLayout(vk.device, pipeline_layout, nullptr);
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Patch_Materials::dispatch(VkCommandBuffer command_buffer, VkDescriptorSet material_descriptor_set) {
    // TEMP: set global sets
    vkCmdBindDescriptorSets(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout, 0, 1, &material_descriptor_set, 0, nullptr);
    // TEMP END

    vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdDispatch(command_buffer, 1, 1, 1);
}
