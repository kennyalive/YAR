#pragma once

#include "common.h"
#include "lib/matrix.h"

struct Rt_Uniform_Buffer;
struct Scene;

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
    VkDescriptorSet             descriptor_set;

    VkPipelineLayout            pipeline_layout;
    VkPipeline                  pipeline;

    Vk_Buffer                   shader_binding_table;

    Vk_Buffer                   uniform_buffer;
    Rt_Uniform_Buffer*          mapped_uniform_buffer;

    Vk_Buffer instance_info_buffer;

    void create(const Scene& scene, const std::vector<GPU_Mesh>& gpu_meshes, VkDescriptorSetLayout material_descriptor_set_layout);
    void destroy();
    void update_output_image_descriptor(VkImageView output_image_view);
    void update_camera_transform(const Matrix3x4& camera_to_world_transform);
    void update_mesh_transform(uint32_t mesh_index, const Matrix3x4& mesh_transform);
    void update_point_lights(VkBuffer light_buffer, int light_count);
    void update_diffuse_rectangular_lights(VkBuffer light_buffer, int light_count);

private:
    void create_acceleration_structure(const std::vector<GPU_Mesh>& gpu_meshes);
    void create_pipeline(const std::vector<GPU_Mesh>& gpu_meshes, VkDescriptorSetLayout material_descriptor_set_layout);
};
