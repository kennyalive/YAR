#pragma once

#include "vk.h"

struct Scene_Object;
struct GPU_Mesh;

struct BLAS_Info {
    VkAccelerationStructureKHR acceleration_structure = VK_NULL_HANDLE;
    Vk_Buffer buffer;
    VkDeviceAddress device_address = 0;
};

struct TLAS_Info {
    VkAccelerationStructureKHR aceleration_structure = VK_NULL_HANDLE;
    Vk_Buffer buffer;
    Vk_Buffer scratch_buffer;
};

struct Vk_Intersection_Accelerator {
    std::vector<BLAS_Info> bottom_level_accels;
    TLAS_Info top_level_accel;
    Vk_Buffer instance_buffer;
    void destroy();
};

Vk_Intersection_Accelerator create_intersection_accelerator(const std::vector<Scene_Object>& scene_objects, const std::vector<GPU_Mesh>& gpu_meshes); 
