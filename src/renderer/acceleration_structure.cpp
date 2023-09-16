#include "std.h"
#include "lib/common.h"

#include "acceleration_structure.h"
#include "geometry.h"
#include "vk.h"

#include "lib/scene_object.h"

// TODO: temp structure. Use separate buffer per attribute.
struct GPU_Vertex {
    Vector3 position;
    Vector3 normal;
    Vector2 uv;
};

static BLAS_Info create_BLAS(const GPU_Mesh& mesh)
{
    VkAccelerationStructureGeometryKHR geometry { VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
    geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;

    auto& trianglesData = geometry.geometry.triangles;
    trianglesData = VkAccelerationStructureGeometryTrianglesDataKHR{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR };
    trianglesData.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
    trianglesData.vertexData.deviceAddress = mesh.vertex_buffer.device_address;
    trianglesData.vertexStride = sizeof(GPU_Vertex);
    trianglesData.maxVertex = mesh.vertex_count - 1;
    trianglesData.indexType = VK_INDEX_TYPE_UINT32;
    trianglesData.indexData.deviceAddress = mesh.index_buffer.device_address;

    VkAccelerationStructureBuildGeometryInfoKHR build_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
    build_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    build_info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    build_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    build_info.geometryCount = 1;
    build_info.pGeometries = &geometry;

    VkAccelerationStructureBuildSizesInfoKHR build_sizes{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
    uint32_t triangle_count = mesh.index_count / 3;
    vkGetAccelerationStructureBuildSizesKHR(vk.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &build_info, &triangle_count, &build_sizes);

    // Create buffer to hold acceleration structure data.
    BLAS_Info blas;
    blas.buffer = vk_create_buffer(build_sizes.accelerationStructureSize, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR, nullptr, "blas_buffer");

    // Create acceleration structure.
    VkAccelerationStructureCreateInfoKHR create_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
    create_info.buffer = blas.buffer.handle;
    create_info.offset = 0;
    create_info.size = build_sizes.accelerationStructureSize;
    create_info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
    VK_CHECK(vkCreateAccelerationStructureKHR(vk.device, &create_info, nullptr, &blas.acceleration_structure));
    vk_set_debug_name(blas.acceleration_structure, "blas");

    // Get acceleration structure address.
    VkAccelerationStructureDeviceAddressInfoKHR device_address_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR };
    device_address_info.accelerationStructure = blas.acceleration_structure;
    blas.device_address = vkGetAccelerationStructureDeviceAddressKHR(vk.device, &device_address_info);

    // Build acceleration structure.
    Vk_Buffer scratch_buffer = vk_create_buffer(build_sizes.buildScratchSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    build_info.dstAccelerationStructure = blas.acceleration_structure;
    build_info.scratchData.deviceAddress = scratch_buffer.device_address;

    VkAccelerationStructureBuildRangeInfoKHR build_range_info{};
    build_range_info.primitiveCount = mesh.index_count / 3;
    const VkAccelerationStructureBuildRangeInfoKHR* p_build_range_infos[1] = { &build_range_info };

    vk_execute(vk.command_pools[0], vk.queue, [&build_info, p_build_range_infos](VkCommandBuffer command_buffer)
    {
        vkCmdBuildAccelerationStructuresKHR(command_buffer, 1, &build_info, p_build_range_infos);
    });
    scratch_buffer.destroy();
    return blas;
}

static TLAS_Info create_TLAS(uint32_t instance_count, VkDeviceAddress instances_device_address)
{
    VkAccelerationStructureGeometryKHR geometry{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR };
    geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
    geometry.geometry.instances = VkAccelerationStructureGeometryInstancesDataKHR{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR };
    geometry.geometry.instances.arrayOfPointers = VK_FALSE;
    geometry.geometry.instances.data.deviceAddress = instances_device_address;

    VkAccelerationStructureBuildGeometryInfoKHR build_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR };
    build_info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    build_info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
    build_info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
    build_info.geometryCount = 1;
    build_info.pGeometries = &geometry;

    VkAccelerationStructureBuildSizesInfoKHR build_sizes{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR };
    vkGetAccelerationStructureBuildSizesKHR(vk.device, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &build_info, &instance_count, &build_sizes);

    // Create buffer to hold acceleration structure data.
    TLAS_Info tlas;
    tlas.buffer = vk_create_buffer(build_sizes.accelerationStructureSize, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR, nullptr, "tlas_buffer");

    // Create acceleration structure.
    VkAccelerationStructureCreateInfoKHR create_info{ VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR };
    create_info.buffer = tlas.buffer.handle;
    create_info.offset = 0;
    create_info.size = build_sizes.accelerationStructureSize;
    create_info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
    VK_CHECK(vkCreateAccelerationStructureKHR(vk.device, &create_info, nullptr, &tlas.aceleration_structure));
    vk_set_debug_name(tlas.aceleration_structure, "tlas");

    // Build acceleration structure.
    tlas.scratch_buffer = vk_create_buffer(build_sizes.buildScratchSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
    build_info.dstAccelerationStructure = tlas.aceleration_structure;
    build_info.scratchData.deviceAddress = tlas.scratch_buffer.device_address;

    VkAccelerationStructureBuildRangeInfoKHR build_range_info{};
    build_range_info.primitiveCount = instance_count;
    const VkAccelerationStructureBuildRangeInfoKHR* p_build_range_infos[1] = { &build_range_info };

    vk_execute(vk.command_pools[0], vk.queue, [&build_info, p_build_range_infos](VkCommandBuffer command_buffer)
    {
        vkCmdBuildAccelerationStructuresKHR(command_buffer, 1, &build_info, p_build_range_infos);
    });
    return tlas;
}

Vk_Intersection_Accelerator create_intersection_accelerator(
    const std::vector<Scene_Object>& scene_objects,
    const std::vector<GPU_Mesh>& gpu_meshes)
{
    Timestamp t;
    Vk_Intersection_Accelerator accelerator;

    // Create BLASes.
    accelerator.bottom_level_accels.resize(gpu_meshes.size());
    for (int i = 0; i < (int)gpu_meshes.size(); i++) {
        accelerator.bottom_level_accels[i] = create_BLAS(gpu_meshes[i]);
    }
    // Create instance buffer.
    {
        std::vector<VkAccelerationStructureInstanceKHR> instances(scene_objects.size());
        for (size_t i = 0; i < instances.size(); i++) {
            memcpy(instances[i].transform.matrix, scene_objects[i].object_to_world_transform.a, sizeof(VkTransformMatrixKHR));
            instances[i].instanceCustomIndex = i;
            instances[i].mask = 0xff;
            instances[i].instanceShaderBindingTableRecordOffset = 0;
            instances[i].flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR;

            // TODO: fix index computation when we have more than one geometry type
            ASSERT(scene_objects[i].geometry.type == Geometry_Type::triangle_mesh);
            int geometry_index = scene_objects[i].geometry.index; 

            instances[i].accelerationStructureReference = accelerator.bottom_level_accels[geometry_index].device_address;
        }
        VkDeviceSize instance_buffer_size = instances.size() * sizeof(VkAccelerationStructureInstanceKHR);
        accelerator.instance_buffer = vk_create_buffer(instance_buffer_size,
            VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            instances.data(), "instance_buffer");
    }
    // Create TLAS.
    accelerator.top_level_accel = create_TLAS((uint32_t)scene_objects.size(), accelerator.instance_buffer.device_address);

    printf("\nAcceleration structures build time = %lld microseconds\n", elapsed_nanoseconds(t) / 1000);
    return accelerator;
}

void Vk_Intersection_Accelerator::destroy()
{
    for (auto& blas : bottom_level_accels) {
        vkDestroyAccelerationStructureKHR(vk.device, blas.acceleration_structure, nullptr);
        blas.buffer.destroy();
    }
    vkDestroyAccelerationStructureKHR(vk.device, top_level_accel.aceleration_structure, nullptr);
    top_level_accel.buffer.destroy();
    top_level_accel.scratch_buffer.destroy();
    instance_buffer.destroy();
    *this = Vk_Intersection_Accelerator{};
}
