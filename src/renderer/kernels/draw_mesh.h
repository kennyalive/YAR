#pragma once

#include "renderer/vk.h"

struct GPU_Mesh;
struct Matrix3x4;
struct Kernel_Context;

namespace GPU_Types { struct Instance_Info; }

struct Draw_Mesh {
    VkDescriptorSetLayout       descriptor_set_layout;
    VkPipelineLayout            pipeline_layout;
    VkPipeline                  pipeline;
    VkDescriptorSet             descriptor_set;

    Vk_Buffer                   uniform_buffer;
    void*                       mapped_uniform_buffer;

    void create(const Kernel_Context& ctx, VkRenderPass render_pass, bool disable_backface_culling, bool front_face_has_clockwsise_winding);
    void destroy();
    void update_point_lights(int light_count);
    void update_directional_lights(int light_count);
    void update_diffuse_rectangular_lights(int light_count);
    void update(const Matrix3x4& view_transform, float fov);

    void bind_sets_and_pipeline();
    void dispatch(const GPU_Mesh& gpu_mesh, int instance_index);
};

