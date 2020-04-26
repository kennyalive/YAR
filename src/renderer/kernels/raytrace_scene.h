#pragma once

#include "renderer/acceleration_structure.h"

struct Matrix3x4;
struct Scene_Object;
struct Rt_Uniform_Buffer;
struct Scene;
struct Kernel_Context;

struct Raytrace_Scene {
    VkPhysicalDeviceRayTracingPropertiesNV properties;
    Vk_Intersection_Accelerator accelerator;
    Vk_Buffer mesh_materials;

    VkDescriptorSetLayout descriptor_set_layout;
    VkDescriptorSet descriptor_set;

    VkPipelineLayout pipeline_layout;
    VkPipeline pipeline;

    Vk_Buffer shader_binding_table;

    Vk_Buffer uniform_buffer;
    Rt_Uniform_Buffer* mapped_uniform_buffer;

    void create(const Kernel_Context& ctx, const Scene& scene, const std::vector<GPU_Mesh>& gpu_meshes);
    void destroy();
    void update_output_image_descriptor(VkImageView output_image_view);
    void update_camera_transform(const Matrix3x4& camera_to_world_transform);
    void update_point_lights(int light_count);
    void update_directional_lights(int light_count);
    void update_diffuse_rectangular_lights(int light_count);

    void dispatch(float fovy, bool spp4, bool z_is_up);

private:
    void create_pipeline(const Kernel_Context& ctx, const std::vector<GPU_Mesh>& gpu_meshes);
};

