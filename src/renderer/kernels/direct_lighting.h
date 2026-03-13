#pragma once

#include "renderer/acceleration_structure.h"

struct Matrix3x4;
struct Rt_Uniform_Buffer;
struct Scene;
struct Descriptor_Heap;
struct Descriptors;

struct Direct_Lighting {
    VkPhysicalDeviceRayTracingPipelinePropertiesKHR properties;
    Vk_Intersection_Accelerator accelerator;
    Vk_Buffer mesh_materials;

    VkPipeline pipeline;

    Vk_Buffer shader_binding_table;

    Vk_Buffer uniform_buffer;
    Rt_Uniform_Buffer* mapped_uniform_buffer;
    uint32_t accelerator_heap_offset = 0;
    uint32_t uniform_buffer_heap_offset = 0;

    void create(Descriptor_Heap& descriptor_heap, const Descriptors& descriptors, const std::vector<VkDescriptorSetAndBindingMappingEXT>& global_heap_mappings, const Scene& scene, const std::vector<GPU_Mesh>& gpu_meshes);
    void destroy();
    void update_camera_transform(const Matrix3x4& camera_to_world_transform);
    void update_point_lights(uint32_t light_count);
    void update_directional_lights(uint32_t light_count);
    void update_diffuse_rectangular_lights(uint32_t light_count);

    void dispatch(float fovy, bool spp4, bool z_is_up);

private:
    void create_pipeline(const Descriptors& descriptors, const std::vector<VkDescriptorSetAndBindingMappingEXT>& global_heap_mappings, const std::vector<GPU_Mesh>& gpu_meshes);
};
