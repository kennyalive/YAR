#pragma once

#include "gpu_structures.h"
#include "vk.h"
#include "io/io.h"
#include "lib/matrix.h"


#include <vector>

struct Rt_Uniform_Buffer;

struct VkGeometryInstanceNV {
    Matrix3x4      transform;
    uint32_t       instanceCustomIndex : 24;
    uint32_t       mask : 8;
    uint32_t       instanceOffset : 24;
    uint32_t       flags : 8;
    uint64_t       accelerationStructureHandle;
};

struct Mesh_Accel {
    VkAccelerationStructureNV accel;
    VmaAllocation allocation;
    uint64_t handle;
};

struct Raytracing_Resources {
    VkPhysicalDeviceRayTracingPropertiesNV properties;

    std::vector<Mesh_Accel> mesh_accels;
    Vk_Buffer mesh_materials;

    VkAccelerationStructureNV top_level_accel;
    VmaAllocation top_level_accel_allocation;

    Vk_Buffer instance_buffer;
    VkGeometryInstanceNV* instance_buffer_ptr;

    VkDescriptorSetLayout       descriptor_set_layout;
    VkPipelineLayout            pipeline_layout;
    VkPipeline                  pipeline;
    VkDescriptorSet             descriptor_set;

    Vk_Buffer                   shader_binding_table;

    Vk_Buffer                   uniform_buffer;
    Rt_Uniform_Buffer*          mapped_uniform_buffer;

    Vk_Buffer material_buffer;
    GPU_Mesh_Material* material_buffer_ptr;

    void create(const Scene_Data& scene, const std::vector<GPU_Mesh>& gpu_meshes);
    void destroy();
    void update_output_image_descriptor(VkImageView output_image_view);
    void update_camera_transform(const Matrix3x4& camera_to_world_transform);
    void update_mesh_transform(uint32_t mesh_index, const Matrix3x4& mesh_transform);
    void update_point_lights(const RGB_Point_Light_Data* point_lights, int point_light_count);

private:
    void create_acceleration_structure(const std::vector<Mesh_Data>& meshes, const std::vector<GPU_Mesh>& gpu_meshes);
    void create_pipeline(const std::vector<GPU_Mesh>& gpu_meshes);
};
