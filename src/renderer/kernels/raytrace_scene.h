#pragma once

#include "renderer/common.h"
#include "renderer/acceleration_structure.h"

#include "lib/matrix.h"

struct Render_Object;
struct Rt_Uniform_Buffer;
struct Scene;


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

    Vk_Buffer instance_info_buffer;

    void create(const Scene& scene, const std::vector<GPU_Mesh>& gpu_meshes, VkDescriptorSetLayout light_descriptor_set_layout, VkDescriptorSetLayout material_descriptor_set_layout, VkDescriptorSetLayout image_descriptor_set_layout);
    void destroy();
    void update_output_image_descriptor(VkImageView output_image_view);
    void update_camera_transform(const Matrix3x4& camera_to_world_transform);
    void update_point_lights(int light_count);
    void update_diffuse_rectangular_lights(int light_count);

    void dispatch(VkDescriptorSet material_descriptor_set, VkDescriptorSet image_descriptor_set, VkDescriptorSet light_descriptor_set, float fovy, bool spp4);

private:
    void create_pipeline(const std::vector<GPU_Mesh>& gpu_meshes, VkDescriptorSetLayout light_descriptor_set_layout, VkDescriptorSetLayout material_descriptor_set_layout, VkDescriptorSetLayout image_descriptor_set_layout);
};

