#include "std.h"
#include "lib/common.h"

#include "common.h"
#include "acceleration_structure.h"
#include "vk.h"

#include "lib/render_object.h"

// TODO: temp structure. Use separate buffer per attribute.
struct GPU_Vertex {
    Vector3 position;
    Vector3 normal;
    Vector2 uv;
};


void Vk_Intersection_Accelerator::destroy() {
    for (VkAccelerationStructureNV accel : bottom_level_accels) {
        vkDestroyAccelerationStructureNV(vk.device, accel, nullptr);
    }
    vkDestroyAccelerationStructureNV(vk.device, top_level_accel, nullptr);
    vmaFreeMemory(vk.allocator, allocation);
    instance_buffer.destroy();
    *this = Vk_Intersection_Accelerator{};
}

static VkGeometryNV create_VkGeometryNV_from_mesh(const GPU_Mesh& gpu_mesh) {
    VkGeometryTrianglesNV triangles{ VK_STRUCTURE_TYPE_GEOMETRY_TRIANGLES_NV };
    triangles.vertexData = gpu_mesh.vertex_buffer.handle;
    triangles.vertexOffset = 0;
    triangles.vertexCount = gpu_mesh.model_vertex_count;
    triangles.vertexStride = sizeof(GPU_Vertex);
    triangles.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
    triangles.indexData = gpu_mesh.index_buffer.handle;
    triangles.indexOffset = 0;
    triangles.indexCount = gpu_mesh.model_index_count;
    triangles.indexType = VK_INDEX_TYPE_UINT32;

    VkGeometryNV geometry{ VK_STRUCTURE_TYPE_GEOMETRY_NV };
    geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_NV;
    geometry.flags = VK_GEOMETRY_OPAQUE_BIT_NV;
    geometry.geometry.triangles = triangles;
    geometry.geometry.aabbs = VkGeometryAABBNV{ VK_STRUCTURE_TYPE_GEOMETRY_AABB_NV };
    return geometry;
}

static VmaAllocation allocate_acceleration_structures_memory(const Vk_Intersection_Accelerator& accelerator) {
    // Get memory requirements for each acceleration structure.
    auto get_memory_reqs = [](VkAccelerationStructureNV accel) {
        VkAccelerationStructureMemoryRequirementsInfoNV reqs_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_INFO_NV };
        reqs_info.type = VK_ACCELERATION_STRUCTURE_MEMORY_REQUIREMENTS_TYPE_OBJECT_NV;
        reqs_info.accelerationStructure = accel;
        VkMemoryRequirements2 reqs_holder{ VK_STRUCTURE_TYPE_MEMORY_REQUIREMENTS_2 };
        vkGetAccelerationStructureMemoryRequirementsNV(vk.device, &reqs_info, &reqs_holder);
        return reqs_holder.memoryRequirements;
    };

    VkMemoryRequirements top_level_memory_reqs = get_memory_reqs(accelerator.top_level_accel);
    std::vector<VkMemoryRequirements> bottom_level_memory_reqs(accelerator.bottom_level_accels.size());
    for (int i = 0; i < (int)accelerator.bottom_level_accels.size(); i++) {
        bottom_level_memory_reqs[i] = get_memory_reqs(accelerator.bottom_level_accels[i]);
    }

    // Compute alignment and memory type bits.
    VkDeviceSize alignment = top_level_memory_reqs.alignment;
    uint32_t memory_type_bits = top_level_memory_reqs.memoryTypeBits;
    for (const VkMemoryRequirements& reqs : bottom_level_memory_reqs) {
        alignment = std::max(alignment, reqs.alignment);
        memory_type_bits &= reqs.memoryTypeBits;
    }
    ASSERT(memory_type_bits != 0); // not guaranteed by spec

    // Compute required amount of memory and offsets.
    std::vector<VkDeviceSize> bottom_level_offsets(accelerator.bottom_level_accels.size());
    VkDeviceSize size = top_level_memory_reqs.size;
    for (int i = 0; i < (int)accelerator.bottom_level_accels.size(); i++) {
        const VkDeviceSize offset = round_up(size, alignment);
        bottom_level_offsets[i] = offset;
        size = offset + bottom_level_memory_reqs[i].size;
    }

    // Allocate memory.
    VmaAllocationCreateInfo alloc_create_info{};
    alloc_create_info.usage = VMA_MEMORY_USAGE_GPU_ONLY;

    VkMemoryRequirements memory_reqs;
    memory_reqs.size = size;
    memory_reqs.alignment = alignment;
    memory_reqs.memoryTypeBits = memory_type_bits;

    VmaAllocation allocation;
    VmaAllocationInfo alloc_info;
    VK_CHECK(vmaAllocateMemory(vk.allocator, &memory_reqs, &alloc_create_info, &allocation, &alloc_info));

    // Bind memory to acceleration structures.
    std::vector<VkBindAccelerationStructureMemoryInfoNV> bind_infos(1 + accelerator.bottom_level_accels.size());
    bind_infos[0] = VkBindAccelerationStructureMemoryInfoNV{ VK_STRUCTURE_TYPE_BIND_ACCELERATION_STRUCTURE_MEMORY_INFO_NV };
    bind_infos[0].accelerationStructure = accelerator.top_level_accel;
    bind_infos[0].memory = alloc_info.deviceMemory;
    bind_infos[0].memoryOffset = alloc_info.offset + 0;

    for (int i = 0; i < (int)accelerator.bottom_level_accels.size(); i++) {
        bind_infos[i+1] = VkBindAccelerationStructureMemoryInfoNV{ VK_STRUCTURE_TYPE_BIND_ACCELERATION_STRUCTURE_MEMORY_INFO_NV };
        bind_infos[i+1].accelerationStructure = accelerator.bottom_level_accels[i];
        bind_infos[i+1].memory = alloc_info.deviceMemory;
        bind_infos[i+1].memoryOffset = alloc_info.offset + bottom_level_offsets[i];
    }
    VK_CHECK(vkBindAccelerationStructureMemoryNV(vk.device, (uint32_t)bind_infos.size(), bind_infos.data()));
    return allocation;
}

Vk_Intersection_Accelerator create_intersection_accelerator(const std::vector<Render_Object>& render_objects, const std::vector<GPU_Mesh>& gpu_meshes) {
    Vk_Intersection_Accelerator accelerator;

    // Create bottom level acceleration structures.
    accelerator.bottom_level_accels.resize(gpu_meshes.size());
    for (int i = 0; i < (int)gpu_meshes.size(); i++) {
        VkGeometryNV geometry = create_VkGeometryNV_from_mesh(gpu_meshes[i]);

        VkAccelerationStructureInfoNV accel_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_INFO_NV };
        accel_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_NV;
        accel_info.geometryCount = 1;
        accel_info.pGeometries = &geometry;

        VkAccelerationStructureCreateInfoNV create_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_NV };
        create_info.info = accel_info;
        VK_CHECK(vkCreateAccelerationStructureNV(vk.device, &create_info, nullptr, &accelerator.bottom_level_accels[i]));
        vk_set_debug_name(accelerator.bottom_level_accels[i], "bottom_level_accel");
    }

    // Create top level acceleration structure.
    {
        VkAccelerationStructureInfoNV accel_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_INFO_NV };
        accel_info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_NV;
        accel_info.instanceCount = (uint32_t)render_objects.size();

        VkAccelerationStructureCreateInfoNV create_info { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_NV };
        create_info.info = accel_info;
        VK_CHECK(vkCreateAccelerationStructureNV(vk.device, &create_info, nullptr, &accelerator.top_level_accel));
        vk_set_debug_name(accelerator.top_level_accel, "top_level_accel");
    }

    // Allocate memory and bind acceleration structures.
    // Single allocation is made, acceleration structures are bound at different offsets.
    accelerator.allocation = allocate_acceleration_structures_memory(accelerator);

    // Create instance buffer.
    {
        std::vector<uint64_t> bottom_level_accel_handles(gpu_meshes.size());
        for (int i = 0; i < (int)gpu_meshes.size(); i++) {
            VK_CHECK(vkGetAccelerationStructureHandleNV(vk.device, accelerator.bottom_level_accels[i], sizeof(uint64_t), &bottom_level_accel_handles[i]));
        }

        std::vector<VkGeometryInstanceNV> instances(render_objects.size());
        for (int i = 0; i < (int)instances.size(); i++) {
            instances[i].transform = render_objects[i].object_to_world_transform;
            instances[i].instanceCustomIndex = i;
            instances[i].mask = 0xff;
            instances[i].instanceOffset = 0;
            instances[i].flags = 0;
            ASSERT(render_objects[i].geometry.type == Geometry_Type::triangle_mesh);
            instances[i].accelerationStructureHandle = bottom_level_accel_handles[render_objects[i].geometry.index];
        }
        VkDeviceSize instance_buffer_size = instances.size() * sizeof(VkGeometryInstanceNV);
        accelerator.instance_buffer = vk_create_buffer(instance_buffer_size, VK_BUFFER_USAGE_RAY_TRACING_BIT_NV | VK_BUFFER_USAGE_TRANSFER_DST_BIT, instances.data(), "instance_buffer");
    }

    // Create scratch buffert for building acceleration structures.
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

        VkDeviceSize scratch_buffer_size = get_scratch_buffer_size(accelerator.top_level_accel);
        for (VkAccelerationStructureNV accel : accelerator.bottom_level_accels) {
            scratch_buffer_size = std::max(scratch_buffer_size, get_scratch_buffer_size(accel));
        }
        scratch_buffer = vk_create_buffer(scratch_buffer_size, VK_BUFFER_USAGE_RAY_TRACING_BIT_NV);
    }

    // Build acceleration structures.
    Timestamp t;

    vk_execute(vk.command_pool, vk.queue,
        [&render_objects, &gpu_meshes, &accelerator, &scratch_buffer](VkCommandBuffer command_buffer)
    {
        VkMemoryBarrier barrier { VK_STRUCTURE_TYPE_MEMORY_BARRIER };
        barrier.srcAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_NV | VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_NV;
        barrier.dstAccessMask = VK_ACCESS_ACCELERATION_STRUCTURE_READ_BIT_NV | VK_ACCESS_ACCELERATION_STRUCTURE_WRITE_BIT_NV;

        VkAccelerationStructureInfoNV bottom_accel_info { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_INFO_NV };
        bottom_accel_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_NV;
        bottom_accel_info.geometryCount = 1;

        for (auto [i, accel] : enumerate(accelerator.bottom_level_accels)) {
            VkGeometryNV geometry = create_VkGeometryNV_from_mesh(gpu_meshes[i]);
            bottom_accel_info.pGeometries = &geometry;
            vkCmdBuildAccelerationStructureNV(command_buffer,
                &bottom_accel_info,
                VK_NULL_HANDLE, // instanceData
                0, // instanceOffset
                VK_FALSE, // update
                accel, // dst
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
            accelerator.instance_buffer.handle, // instanceData
            0, // instanceOffset
            VK_FALSE, // update
            accelerator.top_level_accel, // dst
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
    return accelerator;
}

