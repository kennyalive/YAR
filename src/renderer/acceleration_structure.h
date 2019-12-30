#pragma once

#include "vk.h"

#include "lib/matrix.h"

struct Scene_Object;
struct GPU_Mesh;

// Instance structure definition as recommended by Vulkan spec (33.3.1).
struct VkGeometryInstanceNV {
    Matrix3x4 transform;
    uint32_t instanceCustomIndex : 24;
    uint32_t mask : 8;
    uint32_t instanceOffset : 24;
    uint32_t flags : 8;
    uint64_t accelerationStructureHandle;
};

struct Vk_Intersection_Accelerator {
    std::vector<VkAccelerationStructureNV> bottom_level_accels;
    VkAccelerationStructureNV top_level_accel = VK_NULL_HANDLE;
    VmaAllocation allocation = VK_NULL_HANDLE; // allocation shared by bottom level and top level acceleration structures
    Vk_Buffer instance_buffer; // array of VkGeometryInstanceNV
    void destroy();
};

Vk_Intersection_Accelerator create_intersection_accelerator(const std::vector<Scene_Object>& scene_objects, const std::vector<GPU_Mesh>& gpu_meshes); 

