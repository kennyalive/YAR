#include "std.h"
#include "lib/common.h"
#include "patch_materials.h"

void Patch_Materials::create(const std::vector<VkDescriptorSetAndBindingMappingEXT>& descriptor_mappings)
{
    Vk_Shader_Module shader(get_spirv_file("patch_materials").data());
    pipeline = vk_create_compute_pipeline(shader.handle, descriptor_mappings, "patch_materials_pipeline");
}

void Patch_Materials::destroy()
{
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Patch_Materials::dispatch(VkCommandBuffer command_buffer)
{
    vkCmdBindPipeline(command_buffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
    vkCmdDispatch(command_buffer, 1, 1, 1);
}
