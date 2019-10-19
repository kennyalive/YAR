#pragma once

#include "kernels/copy_to_swapchain.h"
#include "kernels/patch_materials.h"
#include "kernels/raster_resources.h"
#include "kernels/rt_resources.h"
#include "utils.h"
#include "vk.h"

#include "lib/flying_camera.h"
#include "lib/matrix.h"
#include "lib/project.h"
#include "lib/scene.h"

struct GLFWwindow;

class Realtime_Renderer {
public:
    void initialize(Vk_Create_Info vk_create_info, GLFWwindow* window);
    void shutdown();

    void release_resolution_dependent_resources();
    void restore_resolution_dependent_resources();
    bool vsync_enabled() const { return vsync; }
    void toggle_ui() { show_ui = !show_ui; }

    void load_project(const std::string& yar_file_name);
    void run_frame();

private:
    void create_default_textures();
    void draw_frame();
    void draw_rasterized_image();
    void draw_raytraced_image();
    void draw_imgui();
    void copy_output_image_to_swapchain();
    void do_imgui();
    void start_reference_renderer();

private:
    struct UI_Result {
        bool raytracing_toggled;
    };

    bool show_ui = true;
    bool vsync = true;
    bool raytracing = true;
    bool spp4 = false;

    Flying_Camera flying_camera;

    UI_Result                   ui_result;

    VkRenderPass                ui_render_pass;
    VkFramebuffer               ui_framebuffer;
    Vk_Image                    output_image;
    Copy_To_Swapchain           copy_to_swapchain;

    std::vector<GPU_Mesh> gpu_meshes;


    struct GPU_Scene_Resources {
        std::vector<Vk_Image> images_2d;

        Vk_Buffer point_lights;
        Vk_Buffer diffuse_rectangular_lights;

        Vk_Buffer lambertian_material_buffer;
        VkDescriptorSetLayout material_descriptor_set_layout;
        VkDescriptorSet material_descriptor_set;
        VkDescriptorSetLayout image_descriptor_set_layout;
        VkDescriptorSet image_descriptor_set;
    } gpu_scene;

    Patch_Materials patch_materials;
    Rasterization_Resources raster;
    Raytracing_Resources rt;

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
};

