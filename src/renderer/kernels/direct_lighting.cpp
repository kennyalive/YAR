#include "std.h"
#include "lib/common.h"
#include "direct_lighting.h"

#include "renderer/descriptors.h"
#include "renderer/descriptor_heap.h"
#include "renderer/geometry.h"
#include "renderer/vk_utils.h"
#include "shaders/shared_main.slang"
#include "shaders/shared_light.slang"
#include "lib/scene.h"

struct Rt_Uniform_Buffer {
    Matrix3x4   camera_to_world;
    uint32_t    point_light_count;
    uint32_t    directional_light_count;
    uint32_t    diffuse_rectangular_light_count;
    Vector2     pad0;
};

// TODO: temp structure. Use separate buffer per attribute.
struct GPU_Vertex {
    Vector3 position;
    Vector3 normal;
    Vector2 uv;
};

void Direct_Lighting::create(Descriptor_Heap& descriptor_heap, const Descriptors& descriptors, const std::vector<VkDescriptorSetAndBindingMappingEXT>& global_heap_mappings, const Scene& scene, const std::vector<GPU_Mesh>& gpu_meshes) {
    uniform_buffer = vk_create_mapped_buffer(static_cast<VkDeviceSize>(sizeof(Rt_Uniform_Buffer)),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, &(void*&)mapped_uniform_buffer, "rt_uniform_buffer");

    accelerator = create_intersection_accelerator(scene.objects, gpu_meshes);
    accelerator_heap_offset = descriptor_heap.allocate_buffer_descriptor();
    uniform_buffer_heap_offset = descriptor_heap.allocate_buffer_descriptor();

    descriptor_heap.write_acceleration_structure_descriptor(accelerator.top_level_accel.device_address, accelerator_heap_offset);
    descriptor_heap.write_buffer_descriptor(uniform_buffer.range(), VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, uniform_buffer_heap_offset);

    create_pipeline(descriptors, global_heap_mappings, gpu_meshes);

    // Shader binding table.
    {
        uint32_t miss_offset = round_up(properties.shaderGroupHandleSize /* raygen slot*/, properties.shaderGroupBaseAlignment);
        uint32_t hit_offset = round_up(miss_offset + properties.shaderGroupHandleSize /* miss + slot */, properties.shaderGroupBaseAlignment);
        uint32_t sbt_buffer_size = hit_offset + 2 * properties.shaderGroupHandleSize /* chit + shadow ray ahit slots */;

        std::vector<uint8_t> data(sbt_buffer_size);
        VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(vk.device, pipeline, 0, 1, properties.shaderGroupHandleSize, data.data() + 0)); // raygen slot
        VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(vk.device, pipeline, 1, 1, properties.shaderGroupHandleSize, data.data() + miss_offset)); // miss slot
        VK_CHECK(vkGetRayTracingShaderGroupHandlesKHR(vk.device, pipeline, 2, 2, 2 * properties.shaderGroupHandleSize, data.data() + hit_offset)); // hit slot
        shader_binding_table = vk_create_buffer(sbt_buffer_size, VK_BUFFER_USAGE_SHADER_BINDING_TABLE_BIT_KHR | VK_BUFFER_USAGE_TRANSFER_DST_BIT, data.data(), "shader_binding_table");
    }
}

void Direct_Lighting::destroy() {
    uniform_buffer.destroy();
    shader_binding_table.destroy();
    accelerator.destroy();

    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Direct_Lighting::update_camera_transform(const Matrix3x4& camera_to_world_transform) {
    mapped_uniform_buffer->camera_to_world = camera_to_world_transform;
}

void Direct_Lighting::update_point_lights(uint32_t light_count) {
    mapped_uniform_buffer->point_light_count = light_count;
}

void Direct_Lighting::update_directional_lights(uint32_t light_count) {
    mapped_uniform_buffer->directional_light_count = light_count;
}

void Direct_Lighting::update_diffuse_rectangular_lights(uint32_t light_count) {
    mapped_uniform_buffer->diffuse_rectangular_light_count = light_count;
}

void Direct_Lighting::create_pipeline(const Descriptors& descriptors, const std::vector<VkDescriptorSetAndBindingMappingEXT>& global_heap_mappings, const std::vector<GPU_Mesh>& gpu_meshes)
{
    // pipeline
    {
        Shader_Module rgen_shader("spirv/raytrace_scene.rgen.spv");
        Shader_Module miss_shader("spirv/raytrace_scene.miss.spv");
        Shader_Module chit_shader("spirv/raytrace_scene.chit.spv");
        Shader_Module shadow_ray_chit_shader("spirv/raytrace_scene_shadow_ray.chit.spv");

        VkPipelineShaderStageCreateInfo stage_infos[4] {};
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
        const VkDescriptorSetAndBindingMappingEXT raygen_uniform_buffer_mapping = map_binding_to_heap_offset(
            KERNEL_SET_0, 2, VK_SPIRV_RESOURCE_TYPE_UNIFORM_BUFFER_BIT_EXT,
            uniform_buffer_heap_offset
        );

        VkDescriptorSetAndBindingMappingEXT raygen_mappings[3] = {
            raygen_output_image_mapping,
            raygen_accel_mapping,
            raygen_uniform_buffer_mapping
        };

        std::vector<VkDescriptorSetAndBindingMappingEXT> mappings = global_heap_mappings;
        mappings.insert(mappings.begin(), raygen_mappings, raygen_mappings + std::size(raygen_mappings));

        VkShaderDescriptorSetAndBindingMappingInfoEXT mapping_info{ VK_STRUCTURE_TYPE_SHADER_DESCRIPTOR_SET_AND_BINDING_MAPPING_INFO_EXT };
        mapping_info.mappingCount = (uint32_t)mappings.size();
        mapping_info.pMappings = mappings.data();
        stage_infos[0].pNext = &mapping_info;
        stage_infos[2].pNext = &mapping_info;

        VkRayTracingShaderGroupCreateInfoKHR shader_groups[4];
        {
            auto& group = shader_groups[0];
            group = VkRayTracingShaderGroupCreateInfoKHR{ VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
            group.generalShader = 0;
            group.closestHitShader = VK_SHADER_UNUSED_KHR;
            group.anyHitShader = VK_SHADER_UNUSED_KHR;
            group.intersectionShader = VK_SHADER_UNUSED_KHR;
        }
        {
            auto& group = shader_groups[1];
            group = VkRayTracingShaderGroupCreateInfoKHR{ VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_KHR;
            group.generalShader = 1;
            group.closestHitShader = VK_SHADER_UNUSED_KHR;
            group.anyHitShader = VK_SHADER_UNUSED_KHR;
            group.intersectionShader = VK_SHADER_UNUSED_KHR;
        }
        {
            auto& group = shader_groups[2];
            group = VkRayTracingShaderGroupCreateInfoKHR{ VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_KHR };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_KHR;
            group.generalShader = VK_SHADER_UNUSED_KHR;
            group.closestHitShader = 2;
            group.anyHitShader = VK_SHADER_UNUSED_KHR;
            group.intersectionShader = VK_SHADER_UNUSED_KHR;
        }
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
        create_info.flags = VK_PIPELINE_CREATE_RAY_TRACING_NO_NULL_CLOSEST_HIT_SHADERS_BIT_KHR |
                            VK_PIPELINE_CREATE_RAY_TRACING_NO_NULL_MISS_SHADERS_BIT_KHR;
        create_info.stageCount = (uint32_t)std::size(stage_infos);
        create_info.pStages = stage_infos;
        create_info.groupCount = (uint32_t)std::size(shader_groups);
        create_info.pGroups = shader_groups;
        create_info.maxPipelineRayRecursionDepth = 2;
        VK_CHECK(vkCreateRayTracingPipelinesKHR(vk.device, VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &create_info, nullptr, &pipeline));
    }

}

void Direct_Lighting::dispatch(float fovy, bool spp4, bool z_is_up)
{
    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, pipeline);

    float tan_fovy_over2 = std::tan(radians(fovy/2.f));
    uint32_t push_data[3] = { spp4, *reinterpret_cast<uint32_t*>(&tan_fovy_over2), z_is_up };

    VkPushDataInfoEXT push_data_info{ VK_STRUCTURE_TYPE_PUSH_DATA_INFO_EXT };
    push_data_info.data.address = push_data;
    push_data_info.data.size = 12;
    vkCmdPushDataEXT(vk.command_buffer, &push_data_info);

    const VkBuffer sbt = shader_binding_table.handle;
    const uint32_t sbt_slot_size = properties.shaderGroupHandleSize;
    const uint32_t miss_offset = round_up(sbt_slot_size /* raygen slot*/, properties.shaderGroupBaseAlignment);
    const uint32_t hit_offset = round_up(miss_offset + sbt_slot_size /* miss slot */, properties.shaderGroupBaseAlignment);

    VkStridedDeviceAddressRegionKHR raygen_sbt{};
    raygen_sbt.deviceAddress = shader_binding_table.device_address + 0;
    raygen_sbt.stride = sbt_slot_size;
    raygen_sbt.size = sbt_slot_size;

    VkStridedDeviceAddressRegionKHR miss_sbt{};
    miss_sbt.deviceAddress = shader_binding_table.device_address + miss_offset;
    miss_sbt.stride = sbt_slot_size;
    miss_sbt.size = sbt_slot_size;

    VkStridedDeviceAddressRegionKHR chit_sbt{};
    chit_sbt.deviceAddress = shader_binding_table.device_address + hit_offset;
    chit_sbt.stride = sbt_slot_size;
    chit_sbt.size = 2 * sbt_slot_size;

    VkStridedDeviceAddressRegionKHR callable_sbt{};

    vkCmdTraceRaysKHR(vk.command_buffer, &raygen_sbt, &miss_sbt, &chit_sbt, &callable_sbt,
        vk.surface_size.width, vk.surface_size.height, 1);
}
