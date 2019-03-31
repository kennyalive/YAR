#pragma once

#include "copy_to_swapchain.h"
#include "raster_resources.h"
#include "rt_resources.h"
#include "utils.h"
#include "vk.h"

#include "lib/flying_camera.h"
#include "lib/matrix.h"
#include "lib/mesh.h"
#include "lib/io.h"

struct GLFWwindow;

class Realtime_Renderer {
public:
    void initialize(Vk_Create_Info vk_create_info, GLFWwindow* window);
    void shutdown();

    void release_resolution_dependent_resources();
    void restore_resolution_dependent_resources();
    bool vsync_enabled() const { return vsync; }

    void load_project(const std::string& yar_file_name);
    void run_frame();

private:
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
    bool raytracing = false;
    bool spp4 = false;

    bool parallel_reference_rendering = false;

    Flying_Camera flying_camera;

    UI_Result                   ui_result;

    VkRenderPass                ui_render_pass;
    VkFramebuffer               ui_framebuffer;
    Vk_Image                    output_image;
    Copy_To_Swapchain           copy_to_swapchain;

    std::vector<GPU_Mesh>       gpu_meshes;

    Rasterization_Resources     raster;
    Raytracing_Resources        rt;

    GPU_Time_Keeper time_keeper;
    struct {
        GPU_Time_Scope* frame;
        GPU_Time_Scope* draw;
        GPU_Time_Scope* ui;
        GPU_Time_Scope* compute_copy;
    } gpu_times;

    bool project_loaded = false;
    YAR_Project project;
    Scene_Data scene_data;
};
