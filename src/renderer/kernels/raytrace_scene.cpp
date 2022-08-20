#include "std.h"
#include "lib/common.h"
#include "raytrace_scene.h"

#include "renderer/geometry.h"
#include "renderer/kernel_context.h"
#include "renderer/vk_utils.h"
#include "shaders/shared_main.h"
#include "shaders/shared_light.h"
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

void Raytrace_Scene::create(const Kernel_Context& ctx, const Scene& scene, const std::vector<GPU_Mesh>& gpu_meshes) {
    uniform_buffer = vk_create_mapped_buffer(static_cast<VkDeviceSize>(sizeof(Rt_Uniform_Buffer)),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, &(void*&)mapped_uniform_buffer, "rt_uniform_buffer");

    accelerator = create_intersection_accelerator(scene.objects, gpu_meshes);
    create_pipeline(ctx, gpu_meshes);

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

void Raytrace_Scene::destroy() {
    uniform_buffer.destroy();
    shader_binding_table.destroy();
    accelerator.destroy();

    vkDestroyDescriptorSetLayout(vk.device, descriptor_set_layout, nullptr);
    vkDestroyPipelineLayout(vk.device, pipeline_layout, nullptr);
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Raytrace_Scene::update_output_image_descriptor(VkImageView output_image_view) {
    Descriptor_Writes(descriptor_set).storage_image(0, output_image_view);
}

void Raytrace_Scene::update_camera_transform(const Matrix3x4& camera_to_world_transform) {
    mapped_uniform_buffer->camera_to_world = camera_to_world_transform;
}

void Raytrace_Scene::update_point_lights(int light_count) {
    mapped_uniform_buffer->point_light_count = light_count;
}

void Raytrace_Scene::update_directional_lights(int light_count) {
    mapped_uniform_buffer->directional_light_count = light_count;
}

void Raytrace_Scene::update_diffuse_rectangular_lights(int light_count) {
    mapped_uniform_buffer->diffuse_rectangular_light_count = light_count;
}

void Raytrace_Scene::create_pipeline(const Kernel_Context& ctx, const std::vector<GPU_Mesh>& gpu_meshes)
{
    descriptor_set_layout = Descriptor_Set_Layout()
        .storage_image(0, VK_SHADER_STAGE_RAYGEN_BIT_KHR)
        .accelerator(1, VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR)
        .uniform_buffer(2, VK_SHADER_STAGE_RAYGEN_BIT_KHR | VK_SHADER_STAGE_CLOSEST_HIT_BIT_KHR)
        .create("rt_set_layout");

    pipeline_layout = create_pipeline_layout(
        {
            ctx.base_descriptor_set_layout,
            ctx.material_descriptor_set_layout,
            ctx.light_descriptor_set_layout,
            descriptor_set_layout
        },
        {
            // offset 0: spp (samples per pixel)
            // offset 4: fovy
            // offset 8: is_z_up
            VkPushConstantRange{VK_SHADER_STAGE_ALL, 0, Compatible_Layout_Push_Constant_Count * sizeof(uint32_t)}
        },
        "rt_pipeline_layout"
    );

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

        VkRayTracingPipelineCreateInfoKHR create_info{ VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_KHR };
        create_info.flags = VK_PIPELINE_CREATE_RAY_TRACING_NO_NULL_CLOSEST_HIT_SHADERS_BIT_KHR |
                            VK_PIPELINE_CREATE_RAY_TRACING_NO_NULL_MISS_SHADERS_BIT_KHR;
        create_info.stageCount = (uint32_t)std::size(stage_infos);
        create_info.pStages = stage_infos;
        create_info.groupCount = (uint32_t)std::size(shader_groups);
        create_info.pGroups = shader_groups;
        create_info.maxPipelineRayRecursionDepth = 2;
        create_info.layout = pipeline_layout;
        VK_CHECK(vkCreateRayTracingPipelinesKHR(vk.device, VK_NULL_HANDLE, VK_NULL_HANDLE, 1, &create_info, nullptr, &pipeline));
    }

    descriptor_set = allocate_descriptor_set(descriptor_set_layout);
    Descriptor_Writes(descriptor_set)
        .accelerator(1, accelerator.top_level_accel.aceleration_structure)
        .uniform_buffer(2, uniform_buffer.handle, 0, sizeof(Rt_Uniform_Buffer));
}

void Raytrace_Scene::dispatch(float fovy, bool spp4, bool z_is_up)
{
    vkCmdBindDescriptorSets(vk.command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR,
        pipeline_layout, KERNEL_SET_0, 1, &descriptor_set, 0, nullptr);

    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_KHR, pipeline);

    float tan_fovy_over2 = std::tan(radians(fovy/2.f));
    uint32_t push_constants[3] = { spp4, *reinterpret_cast<uint32_t*>(&tan_fovy_over2), z_is_up };
    vkCmdPushConstants(vk.command_buffer, pipeline_layout, VK_SHADER_STAGE_ALL, 0, 12, &push_constants[0]);

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