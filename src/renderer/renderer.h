#pragma once

#include "kernels/copy_to_swapchain.h"
#include "kernels/patch_materials.h"
#include "kernels/draw_mesh.h"
#include "kernels/raytrace_scene.h"
#include "kernel_context.h"

#include "ui/ui.h"

#include "geometry.h"
#include "utils.h"
#include "vk.h"

#include "lib/flying_camera.h"
#include "lib/matrix.h"
#include "lib/project.h"
#include "lib/scene.h"

struct GLFWwindow;

class Renderer {
public:
    void initialize(Vk_Create_Info vk_create_info, GLFWwindow* window);
    void shutdown();

    void release_resolution_dependent_resources();
    void restore_resolution_dependent_resources();
    bool vsync_enabled() const { return ui.vsync; }
    void toggle_ui() { ui.show_ui = !ui.show_ui; }

    void load_project(const std::string& yar_file_name);
    void run_frame();

private:
    void create_render_passes();
    void create_default_textures();
    void draw_frame();
    void draw_rasterized_image();
    void draw_raytraced_image();
    void draw_imgui();
    void copy_output_image_to_swapchain();
    void start_reference_renderer();

private:
    bool raytracing = true;
    bool spp4 = false;

    Flying_Camera flying_camera;

    VkRenderPass ui_render_pass;
    VkFramebuffer ui_framebuffer;
    VkRenderPass raster_render_pass;
    VkFramebuffer raster_framebuffer;

    Vk_Image output_image;

    Kernel_Context kernel_context;

    std::vector<GPU_Mesh> gpu_meshes;

    struct GPU_Scene_Resources {
        std::vector<Vk_Image> images_2d;
        Vk_Buffer instance_info_buffer;

        // Layout for resources that are bound for the most part of the frame.
        VkPipelineLayout per_frame_pipeline_layout;

        VkDescriptorSetLayout base_descriptor_set_layout;
        VkDescriptorSet base_descriptor_set;

        Vk_Buffer point_lights;
        Vk_Buffer diffuse_rectangular_lights;
        VkDescriptorSetLayout light_descriptor_set_layout;
        VkDescriptorSet light_descriptor_set;

        Vk_Buffer lambertian_material_buffer;
        VkDescriptorSetLayout material_descriptor_set_layout;
        VkDescriptorSet material_descriptor_set;
    } gpu_scene;

    Copy_To_Swapchain copy_to_swapchain;
    Draw_Mesh draw_mesh;
    Patch_Materials patch_materials;
    Raytrace_Scene raytrace_scene;

    GPU_Time_Keeper time_keeper;
    struct {
        GPU_Time_Scope* frame;
        GPU_Time_Scope* draw;
        GPU_Time_Scope* ui;
        GPU_Time_Scope* compute_copy;
    } gpu_times;

    bool project_loaded = false;
    YAR_Project project;
    Scene scene;
    UI ui;
};

