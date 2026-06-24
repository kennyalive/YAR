#include "std.h"
#include "lib/common.h"
#include "path_tracing.h"

#include "renderer/descriptors.h"
#include "renderer/descriptor_heap.h"
#include "shaders/shared.slang"
#include "lib/scene.h"

void Path_Tracing::create(Descriptor_Heap& descriptor_heap, const Descriptors& descriptors,
    const std::vector<VkDescriptorSetAndBindingMappingEXT>& global_heap_mappings,
    const Scene& scene, const std::vector<GPU_Mesh>& gpu_meshes)
{
    accelerator = create_intersection_accelerator(scene.objects, gpu_meshes);
    accelerator_heap_offset = descriptor_heap.allocate_buffer_descriptor();
    descriptor_heap.write_acceleration_structure_descriptor(accelerator.top_level_accel.device_address, accelerator_heap_offset);
    create_pipeline(descriptors, global_heap_mappings);

    // Shader binding table.
    {
        const uint32_t handle_size = vk.ray_tracing_pipeline_properties.shaderGroupHandleSize;
        const uint32_t base_alignment = vk.ray_tracing_pipeline_properties.shaderGroupBaseAlignment;

        uint32_t raygen_offset = 0;
        uint32_t miss_offset = round_up(raygen_offset + 1 * handle_size, base_alignment);
        uint32_t hit_offset = round_up(miss_offset + 2 * handle_size, base_alignment);
        uint32_t sbt_buffer_size = hit_offset + 2 * handle_size;

        std::vector<uint8_t> data(sbt_buffer_size, 0);
        VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(vk.device, pipeline, 0, 1, 1 * handle_size, data.data() + raygen_offset));
        VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(vk.device, pipeline, 1, 1, 1 * handle_size, data.data() + miss_offset));
        VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(vk.device, pipeline, 2, 2, 2 * handle_size, data.data() + hit_offset));

        const VkBufferUsageFlags sbt_usage = VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        shader_binding_table = vk_create_buffer(sbt_buffer_size, sbt_usage, data.data(), "shader_binding_table");
    }
}

void Path_Tracing::destroy()
{
    shader_binding_table.destroy();
    accelerator.destroy();
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Path_Tracing::create_pipeline(const Descriptors& descriptors,
    const std::vector<VkDescriptorSetAndBindingMappingEXT>& global_heap_mappings)
{
    // pipeline
    {
        Vk_Shader_Module rgen_shader(get_spirv_file("path_tracing.rgen"));
        Vk_Shader_Module miss_shader(get_spirv_file("path_tracing.miss"));
        Vk_Shader_Module chit_shader(get_spirv_file("path_tracing.chit"));
        Vk_Shader_Module shadow_ray_chit_shader(get_spirv_file("path_tracing_shadow_ray.chit"));

        VkPipelineShaderStageCreateInfo stage_infos[4]{};
        stage_infos[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[0].stage = VK_SHADER_STAGE_RAYGEN_BIT_KHR;
        stage_infos[0].module = rgen_shader.handle;
        stage_infos[0].pName = "main";

        stage_infos[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[1].stage = VK_SHADER_STAGE_MISS_BIT_KHR;
        stage_infos[1].module = miss_shader.handle;
        stage_infos[1].pName = "main";

        stage_infos[2].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[2].stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
        stage_infos[2].module = chit_shader.handle;
        stage_infos[2].pName = "main";

        stage_infos[3].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[3].stage = VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR;
        stage_infos[3].module = shadow_ray_chit_shader.handle;
        stage_infos[3].pName = "main";

        const VkDescriptorSetAndBindingMappingEXT raygen_output_image_mapping = map_binding_to_heap_offset(
            KERNEL_SET_0, 0, VK_SPIRV_RESOURCE_TYPE_READ_WRITE_IMAGE_BIT_EXT,
            descriptors.output_image
        );
        const VkDescriptorSetAndBindingMappingEXT raygen_accel_mapping = map_binding_to_heap_offset(
            KERNEL_SET_0, 1, VK_SPIRV_RESOURCE_TYPE_ACCELERATION_STRUCTURE_BIT_EXT,
            accelerator_heap_offset
        );
        VkDescriptorSetAndBindingMappingEXT raygen_mappings[2] = {
            raygen_output_image_mapping,
            raygen_accel_mapping
        };

        std::vector<VkDescriptorSetAndBindingMappingEXT> mappings = global_heap_mappings;
        mappings.insert(mappings.begin(), raygen_mappings, raygen_mappings + std::size(raygen_mappings));

        VkShaderDescriptorSetAndBindingMappingInfoEXT mapping_info{ VK_STRUCTURE_TYPE_SHADER_DESCRIPTOR_SET_AND_BINDING_MAPPING_INFO_EXT };
        mapping_info.mappingCount = (uint32_t)mappings.size();
        mapping_info.pMappings = mappings.data();
        stage_infos[0].pNext = &mapping_info;
        stage_infos[2].pNext = &mapping_info;

        VkRayTracingShaderGroupCreateInfoKHR shader_groups[4];
        // raygen
        {
            auto& group = shader_groups[0];
            group = VkRayTracingShaderGroupCreateInfoKHR{ VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
            group.generalShader = 0;
            group.closestHitShader = VK_SHADER_UNUSED_KHR;
            group.anyHitShader = VK_SHADER_UNUSED_KHR;
            group.intersectionShader = VK_SHADER_UNUSED_KHR;
        }
        // miss
        {
            auto& group = shader_groups[1];
            group = VkRayTracingShaderGroupCreateInfoKHR{ VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
            group.generalShader = 1;
            group.closestHitShader = VK_SHADER_UNUSED_KHR;
            group.anyHitShader = VK_SHADER_UNUSED_KHR;
            group.intersectionShader = VK_SHADER_UNUSED_KHR;
        }
        // chit
        {
            auto& group = shader_groups[2];
            group = VkRayTracingShaderGroupCreateInfoKHR{ VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
            group.generalShader = VK_SHADER_UNUSED_KHR;
            group.closestHitShader = 2;
            group.anyHitShader = VK_SHADER_UNUSED_KHR;
            group.intersectionShader = VK_SHADER_UNUSED_KHR;
        }
        // shadow ray chit
        {
            auto& group = shader_groups[3];
            group = VkRayTracingShaderGroupCreateInfoKHR{ VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
            group.generalShader = VK_SHADER_UNUSED_KHR;
            group.closestHitShader = 3;
            group.anyHitShader = VK_SHADER_UNUSED_KHR;
            group.intersectionShader = VK_SHADER_UNUSED_KHR;
        }

        VkPipelineCreateFlags2CreateInfo flags_create_info = { VK_STRUCTURE_TYPE_PIPELINE_CREATE_FLAGS_2_CREATE_INFO };
        flags_create_info.flags = VK_PIPELINE_CREATE_2_DESCRIPTOR_HEAP_BIT_EXT;

        VkRayTracingPipelineCreateInfoKHR create_info{ VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR };
        create_info.pNext = &flags_create_info;
        create_info.flags = VK_PIPELINE_CREATE_RAY_TRACING_NO_NULL_CLOSEST_HIT_SHADERS_BIT_KHR;
        create_info.stageCount = (uint32_t)std::size(stage_infos);
        create_info.pStages = stage_infos;
        create_info.groupCount = (uint32_t)std::size(shader_groups);
        create_info.pGroups = shader_groups;
        create_info.maxPipelineRayRecursionDepth = 2;
        VK_CHECK(vkCreateRayTracingPipelinesKHR(vk.device, VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &create_info, nullptr, &pipeline));
    }

}

void Path_Tracing::dispatch()
{
    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, pipeline);

    const uint32_t handle_size = vk.ray_tracing_pipeline_properties.shaderGroupHandleSize;
    const uint32_t base_alignment = vk.ray_tracing_pipeline_properties.shaderGroupBaseAlignment;

    const uint32_t raygen_offset = 0;
    const uint32_t miss_offset = round_up(raygen_offset + 1 * handle_size, base_alignment);
    const uint32_t hit_offset = round_up(miss_offset + 2 * handle_size, base_alignment);

    VkStridedDeviceAddressRegionKHR raygen_sbt{};
    raygen_sbt.deviceAddress = shader_binding_table.device_address + 0;
    raygen_sbt.stride = handle_size;
    raygen_sbt.size = handle_size;

    VkStridedDeviceAddressRegionKHR miss_sbt{};
    miss_sbt.deviceAddress = shader_binding_table.device_address + miss_offset;
    miss_sbt.stride = handle_size;
    miss_sbt.size = 2 * handle_size;

    VkStridedDeviceAddressRegionKHR chit_sbt{};
    chit_sbt.deviceAddress = shader_binding_table.device_address + hit_offset;
    chit_sbt.stride = handle_size;
    chit_sbt.size = 2 * handle_size;

    VkStridedDeviceAddressRegionKHR callable_sbt{};

    vkCmdTraceRaysKHR(vk.command_buffer, &raygen_sbt, &miss_sbt, &chit_sbt, &callable_sbt,
        vk.surface_size.width, vk.surface_size.height, 1);
}
