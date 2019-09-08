#include "std.h"
#include "lib/common.h"
#include "rt_resources.h"

#include "utils.h"
#include "shaders/gpu_types.h"

#include "lib/scene.h"

struct Rt_Uniform_Buffer {
    Matrix3x4   camera_to_world;
    uint32_t    point_light_count;
    uint32_t    diffuse_rectangular_light_count;
    Vector2     pad0;
};

    // TODO: temp structure. Use separate buffer per attribute.
    struct GPU_Vertex {
        Vector3 position;
        Vector3 normal;
        Vector2 uv;
    };

void Raytracing_Resources::create(const Scene& scene, const std::vector<GPU_Mesh>& gpu_meshes, VkDescriptorSetLayout material_descriptor_set_layout) {
    uniform_buffer = vk_create_mapped_buffer(static_cast<VkDeviceSize>(sizeof(Rt_Uniform_Buffer)),
        VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, &(void*&)mapped_uniform_buffer, "rt_uniform_buffer");

    // Material handles.
    {
        std::vector<GPU_Types::Instance_Info> instance_infos(scene.render_objects.size());
        for (auto [i, render_object] : enumerate(scene.render_objects)) {
            instance_infos[i].material = render_object.material;
            instance_infos[i].geometry = render_object.geometry;
            // TODO: this should be Light_Handle not just light_index, since we could have multiple types of area lights. 
            instance_infos[i].area_light_index = render_object.area_light.index;
            instance_infos[i].pad0 = 0.f;
            instance_infos[i].pad1 = 0.f;
            instance_infos[i].pad2 = 0.f;
        }
        VkDeviceSize size = scene.render_objects.size() * sizeof(GPU_Types::Instance_Info); 
        instance_info_buffer = vk_create_buffer(size, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            instance_infos.data(), "instance_info_buffer");
    }

    // Instance buffer.
    {
        VkDeviceSize instance_buffer_size = scene.render_objects.size() * sizeof(VkGeometryInstanceNV);
        void* buffer_ptr;
        instance_buffer = vk_create_mapped_buffer(instance_buffer_size, VK_BUFFER_USAGE_RAY_TRACING_BIT_NV, &buffer_ptr, "instance_buffer");
        instance_buffer_ptr = static_cast<VkGeometryInstanceNV*>(buffer_ptr);
    }

    create_acceleration_structure(scene.render_objects, gpu_meshes);
    create_pipeline(gpu_meshes, material_descriptor_set_layout);

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

void Raytracing_Resources::destroy() {
    uniform_buffer.destroy();
    instance_info_buffer.destroy();
    shader_binding_table.destroy();

    for (const Mesh_Accel& mesh_accel : mesh_accels) {
        vkDestroyAccelerationStructureNV(vk.device, mesh_accel.accel, nullptr);
        vmaFreeMemory(vk.allocator, mesh_accel.allocation);
    }

    vkDestroyAccelerationStructureNV(vk.device, top_level_accel, nullptr);
    vmaFreeMemory(vk.allocator, top_level_accel_allocation);
    instance_buffer.destroy();
    vkDestroyDescriptorSetLayout(vk.device, descriptor_set_layout, nullptr);

    vkDestroyPipelineLayout(vk.device, pipeline_layout, nullptr);
    vkDestroyPipeline(vk.device, pipeline, nullptr);
}

void Raytracing_Resources::update_output_image_descriptor(VkImageView output_image_view) {
    Descriptor_Writes(descriptor_set).storage_image(0, output_image_view);
}

void Raytracing_Resources::update_camera_transform(const Matrix3x4& camera_to_world_transform) {
    mapped_uniform_buffer->camera_to_world = camera_to_world_transform;
}

void Raytracing_Resources::update_instance_transform(uint32_t mesh_index, uint32_t instance_index, const Matrix3x4& instance_transform) {
    VkGeometryInstanceNV& instance = instance_buffer_ptr[instance_index];
    instance.transform = instance_transform;
    instance.instanceCustomIndex = instance_index;
    instance.mask = 0xff;
    instance.instanceOffset = 0;
    instance.flags = 0;
    instance.accelerationStructureHandle = mesh_accels[mesh_index].handle;
}

void Raytracing_Resources::update_point_lights(VkBuffer light_buffer, int light_count) {
    Descriptor_Writes(descriptor_set).storage_buffer(5, light_buffer, 0, VK_WHOLE_SIZE);
    mapped_uniform_buffer->point_light_count = light_count;
}

void Raytracing_Resources::update_diffuse_rectangular_lights(VkBuffer light_buffer, int light_count) {
    Descriptor_Writes(descriptor_set).storage_buffer(6, light_buffer, 0, VK_WHOLE_SIZE);
    mapped_uniform_buffer->diffuse_rectangular_light_count = light_count;
}

void Raytracing_Resources::create_acceleration_structure(const std::vector<Render_Object>& render_objects, const std::vector<GPU_Mesh>& gpu_meshes) {
    // Initialize VKGeometryNV structures.
    std::vector<VkGeometryNV> geometries(gpu_meshes.size());
    for (auto [i, geom] : enumerate(geometries)) {
        geom = VkGeometryNV { VK_STRUCTURE_TYPE_GEOMETRY_NV };
        geom.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_NV;
        geom.geometry.aabbs = VkGeometryAABBNV { VK_STRUCTURE_TYPE_GEOMETRY_AABB_NV };
        geom.flags = VK_GEOMETRY_OPAQUE_BIT_NV;

        VkGeometryTrianglesNV& triangle_geom = geom.geometry.triangles;
        triangle_geom = VkGeometryTrianglesNV { VK_STRUCTURE_TYPE_GEOMETRY_TRIANGLES_NV };
        triangle_geom.vertexData = gpu_meshes[i].vertex_buffer.handle;
        triangle_geom.vertexOffset = 0;
        triangle_geom.vertexCount = gpu_meshes[i].model_vertex_count;
        triangle_geom.vertexStride = sizeof(GPU_Vertex);
        triangle_geom.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
        triangle_geom.indexData = gpu_meshes[i].index_buffer.handle;
        triangle_geom.indexOffset = 0;
        triangle_geom.indexCount = gpu_meshes[i].model_index_count;
        triangle_geom.indexType = VK_INDEX_TYPE_UINT32;
    }

    // Create acceleration structures and allocate/bind memory.
    {
        auto allocate_acceleration_structure_memory = [](VkAccelerationStructureNV acceleration_structure, VmaAllocation* allocation) {
            VkAccelerationStructureMemoryRequirementsInfoNV reqs_info { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_INFO_NV };
            reqs_info.type = VK_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_TYPE_OBJECT_NV;
            reqs_info.accelerationStructure = acceleration_structure;

            VkMemoryRequirements2 reqs_holder { VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2 };
            vkGetAccelerationStructureMemoryRequirementsNV(vk.device, &reqs_info, &reqs_holder);

            VmaAllocationCreateInfo alloc_create_info{};
            alloc_create_info.usage = VMA_MEMORY_USAGE_GPU_ONLY;

            VmaAllocationInfo alloc_info;
            VK_CHECK(vmaAllocateMemory(vk.allocator, &reqs_holder.memoryRequirements, &alloc_create_info, allocation, &alloc_info));

            VkBindAccelerationStructureMemoryInfoNV bind_info { VK_STRUCTURE_TYPE_BIND_ACCELERATION_STRUCTURE_MEMORY_INFO_NV };
            bind_info.accelerationStructure = acceleration_structure;
            bind_info.memory = alloc_info.deviceMemory;
            bind_info.memoryOffset = alloc_info.offset;
            VK_CHECK(vkBindAccelerationStructureMemoryNV(vk.device, 1, &bind_info));
        };

        // Bottom level.
        mesh_accels.resize(gpu_meshes.size());
        for (auto [i, geometry] : enumerate(geometries)) {
            VkAccelerationStructureInfoNV accel_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_INFO_NV };
            accel_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_NV;
            accel_info.geometryCount = 1;
            accel_info.pGeometries = &geometry;

            VkAccelerationStructureCreateInfoNV create_info { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_NV };
            create_info.info = accel_info;

            VK_CHECK(vkCreateAccelerationStructureNV(vk.device, &create_info, nullptr, &mesh_accels[i].accel));
            allocate_acceleration_structure_memory(mesh_accels[i].accel, &mesh_accels[i].allocation);
            vk_set_debug_name(mesh_accels[i].accel, ("mesh_accel " + std::to_string(i)).c_str());

            VK_CHECK(vkGetAccelerationStructureHandleNV(vk.device, mesh_accels[i].accel, sizeof(uint64_t), &mesh_accels[i].handle));
        }

        // Top level.
        {
            VkAccelerationStructureInfoNV accel_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_INFO_NV };
            accel_info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_NV;
            accel_info.instanceCount = (uint32_t)render_objects.size();

            VkAccelerationStructureCreateInfoNV create_info { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_NV };
            create_info.info = accel_info;

            VK_CHECK(vkCreateAccelerationStructureNV(vk.device, &create_info, nullptr, &top_level_accel));
            allocate_acceleration_structure_memory(top_level_accel, &top_level_accel_allocation);
            vk_set_debug_name(top_level_accel, "top_level_accel");
        }
    }

    for (auto [i, render_object] : enumerate(render_objects)) {
        ASSERT(render_object.geometry.type == Geometry_Type::triangle_mesh);
        update_instance_transform(render_object.geometry.index, (uint32_t)i, render_object.object_to_world_transform);
    }

    // Create scratch buffert required to build acceleration structures.
    Vk_Buffer scratch_buffer;
    {
        auto get_scratch_buffer_size = [](VkAccelerationStructureNV accel) {
            VkAccelerationStructureMemoryRequirementsInfoNV reqs_info { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_INFO_NV };
            reqs_info.type = VK_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_TYPE_BUILD_SCRATCH_NV;
            reqs_info.accelerationStructure = accel;

            VkMemoryRequirements2 reqs_holder;
            vkGetAccelerationStructureMemoryRequirementsNV(vk.device, &reqs_info, &reqs_holder);
            return reqs_holder.memoryRequirements.size;            
        };

        VkDeviceSize scratch_buffer_size = get_scratch_buffer_size(top_level_accel);
        for (const Mesh_Accel& mesh_accel : mesh_accels) {
            scratch_buffer_size = std::max(scratch_buffer_size, get_scratch_buffer_size(mesh_accel.accel));
        }
        scratch_buffer = vk_create_buffer(scratch_buffer_size, VK_BUFFER_USAGE_RAY_TRACING_BIT_NV);
    }

    // Build acceleration structures.
    Timestamp t;

    vk_execute(vk.command_pool, vk.queue,
        [this, &geometries, &render_objects, &scratch_buffer](VkCommandBuffer command_buffer)
    {
        VkMemoryBarrier barrier { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
        barrier.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_NV | VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_NV;
        barrier.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_NV | VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_NV;

        VkAccelerationStructureInfoNV bottom_accel_info { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_INFO_NV };
        bottom_accel_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_NV;
        bottom_accel_info.geometryCount = 1;

        for (auto [i, mesh_accel] : enumerate(mesh_accels)) {
            bottom_accel_info.pGeometries = &geometries[i];
            vkCmdBuildAccelerationStructureNV(command_buffer,
                &bottom_accel_info,
                VK_NULL_HANDLE, // instanceData
                0, // instanceOffset
                VK_FALSE, // update
                mesh_accel.accel, // dst
                VK_NULL_HANDLE, // src
                scratch_buffer.handle,
                0 // scratch_offset
            );

            vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_NV, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_NV,
                0, 1, &barrier, 0, nullptr, 0, nullptr);
        }

        VkAccelerationStructureInfoNV top_accel_info { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_INFO_NV };
        top_accel_info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_NV;
        top_accel_info.instanceCount = (uint32_t)render_objects.size();

        vkCmdBuildAccelerationStructureNV(command_buffer,
            &top_accel_info,
            instance_buffer.handle, // instanceData
            0, // instanceOffset
            VK_FALSE, // update
            top_level_accel, // dst
            VK_NULL_HANDLE, // src
            scratch_buffer.handle, 
            0 // scratch_offset
        );

        barrier.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_NV | VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_NV;
        barrier.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_NV;

        vkCmdPipelineBarrier(command_buffer, VK_PIPELINE_STAGE_ACCELERATION_STRUCTURE_BUILD_BIT_NV, VK_PIPELINE_STAGE_RAY_TRACING_SHADER_BIT_NV,
            0, 1, &barrier, 0, nullptr, 0, nullptr);
    });

    scratch_buffer.destroy();
    printf("\nAcceleration structures build time = %lld microseconds\n", elapsed_microseconds(t));
}

void Raytracing_Resources::create_pipeline(const std::vector<GPU_Mesh>& gpu_meshes, VkDescriptorSetLayout material_descriptor_set_layout) {

    descriptor_set_layout = Descriptor_Set_Layout()
        .storage_image(0, VK_SHADER_STAGE_RAYGEN_BIT_NV)
        .accelerator(1, VK_SHADER_STAGE_RAYGEN_BIT_NV | VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV)
        .uniform_buffer(2, VK_SHADER_STAGE_RAYGEN_BIT_NV | VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV)
        .storage_buffer_array(3, (uint32_t)gpu_meshes.size(), VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV) // index buffers
        .storage_buffer_array(4, (uint32_t)gpu_meshes.size(), VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV) // vertex buffers
        .storage_buffer(5, VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV) // point light buffer
        .storage_buffer(6, VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV) // diffuse rectangular light buffer
        .storage_buffer(7, VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV) // material handle buffer
        .create("rt_set_layout");

    // pipeline layout
    {
        VkPushConstantRange push_constant_ranges[1];
        // offset 0: spp (samples per pixel)
        // offset 4: fovy
        push_constant_ranges[0].stageFlags = VK_SHADER_STAGE_RAYGEN_BIT_NV;
        push_constant_ranges[0].offset = 0;
        push_constant_ranges[0].size = 8;

        VkDescriptorSetLayout set_layouts[] = {
            descriptor_set_layout,
            material_descriptor_set_layout
        };

        VkPipelineLayoutCreateInfo create_info { VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO };
        create_info.setLayoutCount = (uint32_t)std::size(set_layouts);
        create_info.pSetLayouts = set_layouts;
        create_info.pushConstantRangeCount = (uint32_t)std::size(push_constant_ranges);
        create_info.pPushConstantRanges = push_constant_ranges;
        VK_CHECK(vkCreatePipelineLayout(vk.device, &create_info, nullptr, &pipeline_layout));
    }

    // pipeline
    {
        VkShaderModule rgen_shader = vk_load_spirv("spirv/rt_mesh.rgen.spv");
        VkShaderModule miss_shader = vk_load_spirv("spirv/rt_mesh.rmiss.spv");
        VkShaderModule chit_shader = vk_load_spirv("spirv/rt_mesh.rchit.spv");
        VkShaderModule shadow_ray_chit_shader = vk_load_spirv("spirv/rt_shadow_ray.rchit.spv");

        VkPipelineShaderStageCreateInfo stage_infos[4] {};
        stage_infos[0].sType    = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[0].stage    = VK_SHADER_STAGE_RAYGEN_BIT_NV;
        stage_infos[0].module   = rgen_shader;
        stage_infos[0].pName    = "main";

        stage_infos[1].sType    = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[1].stage    = VK_SHADER_STAGE_MISS_BIT_NV;
        stage_infos[1].module   = miss_shader;
        stage_infos[1].pName    = "main";

        stage_infos[2].sType    = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[2].stage    = VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV;
        stage_infos[2].module   = chit_shader;
        stage_infos[2].pName    = "main";

        stage_infos[3].sType    = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        stage_infos[3].stage    = VK_SHADER_STAGE_CLOSEST_HIT_BIT_NV;
        stage_infos[3].module   = shadow_ray_chit_shader;
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

        vkDestroyShaderModule(vk.device, rgen_shader, nullptr);
        vkDestroyShaderModule(vk.device, miss_shader, nullptr);
        vkDestroyShaderModule(vk.device, chit_shader, nullptr);
        vkDestroyShaderModule(vk.device, shadow_ray_chit_shader, nullptr);
    }

    // descriptor set
    {
        VkDescriptorSetAllocateInfo desc { VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO };
        desc.descriptorPool = vk.descriptor_pool;
        desc.descriptorSetCount = 1;
        desc.pSetLayouts = &descriptor_set_layout;
        VK_CHECK(vkAllocateDescriptorSets(vk.device, &desc, &descriptor_set));

        std::vector<VkDescriptorBufferInfo> vertex_buffer_infos(gpu_meshes.size());
        std::vector<VkDescriptorBufferInfo> index_buffer_infos(gpu_meshes.size());

        for (auto [i, gpu_mesh] : enumerate(gpu_meshes)) {
            vertex_buffer_infos[i].buffer = gpu_mesh.vertex_buffer.handle;
            vertex_buffer_infos[i].offset = 0;
            vertex_buffer_infos[i].range = gpu_mesh.model_vertex_count * sizeof(GPU_Vertex);

            index_buffer_infos[i].buffer = gpu_mesh.index_buffer.handle;
            index_buffer_infos[i].offset = 0;
            index_buffer_infos[i].range = gpu_mesh.model_index_count * sizeof(uint32_t);
        }

        Descriptor_Writes(descriptor_set)
            .accelerator(1, top_level_accel)
            .uniform_buffer(2, uniform_buffer.handle, 0, sizeof(Rt_Uniform_Buffer))
            .storage_buffer_array(3, (uint32_t)gpu_meshes.size(), index_buffer_infos.data())
            .storage_buffer_array(4, (uint32_t)gpu_meshes.size(), vertex_buffer_infos.data())
            .storage_buffer(7, instance_info_buffer.handle, 0, VK_WHOLE_SIZE);
    }
}
