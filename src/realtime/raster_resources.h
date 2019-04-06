#pragma once

#include "vk.h"

struct Matrix3x4;
struct RGB_Point_Light_Data;
struct RGB_Diffuse_Rectangular_Light_Data;

struct Rasterization_Resources {
    VkDescriptorSetLayout       descriptor_set_layout;
    VkPipelineLayout            pipeline_layout;
    VkPipeline                  pipeline;
    VkDescriptorSet             descriptor_set;

    VkRenderPass                render_pass;
    VkFramebuffer               framebuffer;

    Vk_Buffer                   uniform_buffer;
    void*                       mapped_uniform_buffer;

    void create();
    void destroy();
    void create_framebuffer(VkImageView output_image_view);
    void destroy_framebuffer();
    void update_point_lights(const RGB_Point_Light_Data* point_lights, int point_light_count);
    void update_diffuse_rectangular_lights(const RGB_Diffuse_Rectangular_Light_Data* lights, int light_count);
    void update(const Matrix3x4& view_transform);
};
