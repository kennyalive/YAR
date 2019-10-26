#pragma once

#include "renderer/vk.h"

struct Matrix3x4;
struct RGB_Point_Light_Data;
struct RGB_Diffuse_Rectangular_Light_Data;

struct Draw_Mesh {
    VkDescriptorSetLayout       descriptor_set_layout;
    VkPipelineLayout            pipeline_layout;
    VkPipeline                  pipeline;
    VkDescriptorSet             descriptor_set;

    Vk_Buffer                   uniform_buffer;
    void*                       mapped_uniform_buffer;

    void create(VkRenderPass render_pass, VkDescriptorSetLayout material_descriptor_set_layout, VkDescriptorSetLayout image_descriptor_set_layout, VkDescriptorSetLayout light_descriptor_set_layout, bool front_face_has_clockwsise_winding);
    void destroy();
    void update_point_lights(int light_count);
    void update_diffuse_rectangular_lights(int light_count);
    void update(const Matrix3x4& view_transform, float fov);
};
