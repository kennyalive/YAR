#include "std.h"
#include "lib/common.h"
#include "raytrace_scene.h"

#include "renderer/geometry.h"
#include "renderer/kernel_context.h"
#include "renderer/utils.h"
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

        void* mapped_memory;
        shader_binding_table = vk_create_mapped_buffer(sbt_buffer_size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, &mapped_memory, "shader_binding_table");

        // raygen slot
        VK_CHECK(vkGetRayTracingShaderGroupHandlesNV(vk.device, pipeline, 0, 1, properties.shaderGroupHandleSize, mapped_memory));

        // miss slot
        VK_CHECK(vkGetRayTracingShaderGroupHandlesNV(vk.device, pipeline, 1, 1, properties.shaderGroupHandleSize, (uint8_t*)mapped_memory + miss_offset));

        // chit + shadow ray chit slots
        VK_CHECK(vkGetRayTracingShaderGroupHandlesNV(vk.device, pipeline, 2, 2, 2*properties.shaderGroupHandleSize, (uint8_t*)mapped_memory + hit_offset));
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

void Raytrace_Scene::dispatch(float fovy, bool spp4, bool z_is_up) {
    vkCmdBindDescriptorSets(vk.command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_NV, pipeline_layout, KERNEL_SET_0, 1, &descriptor_set, 0, nullptr);

    float tan_fovy_over2 = std::tan(radians(fovy/2.f));
    uint32_t push_constants[3] = { spp4, *reinterpret_cast<uint32_t*>(&tan_fovy_over2), z_is_up };
    vkCmdPushConstants(vk.command_buffer, pipeline_layout, VK_SHADER_STAGE_ALL, 0, 12, &push_constants[0]);

    vkCmdBindPipeline(vk.command_buffer, VK_PIPELINE_BIND_POINT_RAY_TRACING_NV, pipeline);

    const VkBuffer sbt = shader_binding_table.handle;
    const uint32_t sbt_slot_size = properties.shaderGroupHandleSize;
    const uint32_t miss_offset = round_up(sbt_slot_size /* raygen slot*/, properties.shaderGroupBaseAlignment);
    const uint32_t hit_offset = round_up(miss_offset + sbt_slot_size /* miss slot */, properties.shaderGroupBaseAlignment);

    vkCmdTraceRaysNV(vk.command_buffer,
        sbt, 0, // raygen shader
        sbt, miss_offset, sbt_slot_size, // miss shader
        sbt, hit_offset, sbt_slot_size, // chit shader
        VK_NULL_HANDLE, 0, 0,
        vk.surface_size.width, vk.surface_size.height, 1);
}

void Raytrace_Scene::create_pipeline(const Kernel_Context& ctx, const std::vector<GPU_Mesh>& gpu_meshes) {

    descriptor_set_layout = Descriptor_Set_Layout()
        .storage_image(0, VK_SHADER_STAGE_RAYGEN_BIT_NV)
        .accelerator(1, VK_SHADER_STAGE_RAYGEN_BIT_NV | VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV)
        .uniform_buffer(2, VK_SHADER_STAGE_RAYGEN_BIT_NV | VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV)
        .create("rt_set_layout");

    // pipeline layout
    {
        VkDescriptorSetLayout set_layouts[] = {
            ctx.base_descriptor_set_layout,
            ctx.material_descriptor_set_layout,
            ctx.light_descriptor_set_layout,
            descriptor_set_layout
        };

        VkPushConstantRange push_constant_ranges[1];
        // offset 0: spp (samples per pixel)
        // offset 4: fovy
        // offset 8: is_z_up
        push_constant_ranges[0].stageFlags = VK_SHADER_STAGE_ALL;
        push_constant_ranges[0].offset = 0;
        push_constant_ranges[0].size = Compatible_Layout_Push_Constant_Count * sizeof(uint32_t);

        VkPipelineLayoutCreateInfo create_info { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        create_info.setLayoutCount = (uint32_t)std::size(set_layouts);
        create_info.pSetLayouts = set_layouts;
        create_info.pushConstantRangeCount = (uint32_t)std::size(push_constant_ranges);
        create_info.pPushConstantRanges = push_constant_ranges;
        VK_CHECK(vkCreatePipelineLayout(vk.device, &create_info, nullptr, &pipeline_layout));
    }

    // pipeline
    {
        Shader_Module rgen_shader("spirv/raytrace_scene.rgen.spv");
        Shader_Module miss_shader("spirv/raytrace_scene.rmiss.spv");
        Shader_Module chit_shader("spirv/raytrace_scene.rchit.spv");
        Shader_Module shadow_ray_chit_shader("spirv/raytrace_scene_shadow_ray.rchit.spv");

        VkPipelineShaderStageCreateInfo stage_infos[4] {};
        stage_infos[0].sType    = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[0].stage    = VK_SHADER_STAGE_RAYGEN_BIT_NV;
        stage_infos[0].module   = rgen_shader.handle;
        stage_infos[0].pName    = "main";

        stage_infos[1].sType    = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[1].stage    = VK_SHADER_STAGE_MISS_BIT_NV;
        stage_infos[1].module   = miss_shader.handle;
        stage_infos[1].pName    = "main";

        stage_infos[2].sType    = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[2].stage    = VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV;
        stage_infos[2].module   = chit_shader.handle;
        stage_infos[2].pName    = "main";

        stage_infos[3].sType    = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[3].stage    = VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV;
        stage_infos[3].module   = shadow_ray_chit_shader.handle;
        stage_infos[3].pName    = "main";

        VkRayTracingShaderGroupCreateInfoNV shader_groups[4];

        {
            auto& group = shader_groups[0];
            group = VkRayTracingShaderGroupCreateInfoNV { VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_NV };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_NV;
            group.generalShader = 0;
            group.closestHitShader = VK_SHADER_UNUSED_NV;
            group.anyHitShader = VK_SHADER_UNUSED_NV;
            group.intersectionShader = VK_SHADER_UNUSED_NV;
        }
        {
            auto& group = shader_groups[1];
            group = VkRayTracingShaderGroupCreateInfoNV { VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_NV };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_GENERAL_NV;
            group.generalShader = 1;
            group.closestHitShader = VK_SHADER_UNUSED_NV;
            group.anyHitShader = VK_SHADER_UNUSED_NV;
            group.intersectionShader = VK_SHADER_UNUSED_NV;
        }
        {
            auto& group = shader_groups[2];
            group = VkRayTracingShaderGroupCreateInfoNV { VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_NV };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_NV;
            group.generalShader = VK_SHADER_UNUSED_NV;
            group.closestHitShader = 2;
            group.anyHitShader = VK_SHADER_UNUSED_NV;
            group.intersectionShader = VK_SHADER_UNUSED_NV;
        }
        {
            auto& group = shader_groups[3];
            group = VkRayTracingShaderGroupCreateInfoNV { VK_STRUCTURE_TYPE_RAY_TRACING_SHADER_GROUP_CREATE_INFO_NV };
            group.type = VK_RAY_TRACING_SHADER_GROUP_TYPE_TRIANGLES_HIT_GROUP_NV;
            group.generalShader = VK_SHADER_UNUSED_NV;
            group.closestHitShader = 3;
            group.anyHitShader = VK_SHADER_UNUSED_NV;
            group.intersectionShader = VK_SHADER_UNUSED_NV;
        }

        VkRayTracingPipelineCreateInfoNV create_info { VK_STRUCTURE_TYPE_RAY_TRACING_PIPELINE_CREATE_INFO_NV };
        create_info.stageCount          = (uint32_t)std::size(stage_infos);
        create_info.pStages             = stage_infos;
        create_info.groupCount          = (uint32_t)std::size(shader_groups);
        create_info.pGroups             = shader_groups;
        create_info.maxRecursionDepth   = 2;
        create_info.layout              = pipeline_layout;
        VK_CHECK(vkCreateRayTracingPipelinesNV(vk.device, VK_NULL_HANDLE, 1, &create_info, nullptr, &pipeline));
    }

    // descriptor set
    {
        VkDescriptorSetAllocateInfo desc { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        desc.descriptorPool = vk.descriptor_pool;
        desc.descriptorSetCount = 1;
        desc.pSetLayouts = &descriptor_set_layout;
        VK_CHECK(vkAllocateDescriptorSets(vk.device, &desc, &descriptor_set));

        Descriptor_Writes(descriptor_set)
            .accelerator(1, accelerator.top_level_accel)
            .uniform_buffer(2, uniform_buffer.handle, 0, sizeof(Rt_Uniform_Buffer));
    }
}
