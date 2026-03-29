#include "std.h"
#include "lib/common.h"
#include "patch_materials.h"

#include "renderer/descriptors.h"

void Patch_Materials::create(Descriptors& descriptors)
{
    const VkDescriptorSetAndBindingMappingEXT mapping = map_binding_to_heap_offset(
        0, 0, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_STORAGE_BUFFER_BIT_EXT,
        descriptors.lambertian_materials
    );
    Vk_Shader_Module shader(get_spirv_file("patch_materials"));
    pipeline = vk_create_compute_pipeline(shader.handle, std::span(&mapping, 1), "patch_materials_pipeline");
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
