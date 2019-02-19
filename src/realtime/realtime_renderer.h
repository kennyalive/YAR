#pragma once

#include "copy_to_swapchain.h"
#include "raster_resources.h"
#include "rt_resources.h"
#include "utils.h"
#include "vk.h"

#include "lib/matrix.h"
#include "lib/mesh.h"
#include "io/io.h"

#include <thread>
#include <vector>

struct GLFWwindow;

class Realtime_Renderer {
public:
    void initialize(Vk_Create_Info vk_create_info, GLFWwindow* window);
    void shutdown();

    void release_resolution_dependent_resources();
    void restore_resolution_dependent_resources();
    bool vsync_enabled() const { return vsync; }

    void run_frame();

private:
    void draw_frame();
    void draw_rasterized_image();
    void draw_raytraced_image();
    void draw_imgui();
    void copy_output_image_to_swapchain();
    void do_imgui();

private:
    struct UI_Result {
        bool raytracing_toggled;
    };

    using Clock = std::chrono::high_resolution_clock;
    using Time  = std::chrono::time_point<Clock>;

    bool                        show_ui                 = true;
    bool                        vsync                   = true;
    bool                        raytracing              = false;
    bool                        spp4                    = false;
    bool                        reference_render_active = false;

    std::thread                 reference_render_thread;
    Matrix3x4                   camera_to_world_transform;

    UI_Result                   ui_result;

    VkRenderPass                ui_render_pass;
    VkFramebuffer               ui_framebuffer;
    Vk_Image                    output_image;
    Copy_To_Swapchain           copy_to_swapchain;

    
    Scene_Data                  scene_data;
    std::vector<GPU_Mesh>       gpu_meshes;

    Vector3                     camera_pos = Vector3(0, 3, 1);
    float                       camera_yaw = -Pi/2; // relative to x axis
    Vector3                     camera_dir = Vector3(0, -1, 0);
    Matrix3x4                   model_transform;
    Matrix3x4                   view_transform;

    Rasterization_Resources     raster;
    Raytracing_Resources        rt;

    GPU_Time_Keeper             time_keeper;
    struct {
        GPU_Time_Interval*      frame;
        GPU_Time_Interval*      draw;
        GPU_Time_Interval*      ui;
        GPU_Time_Interval*      compute_copy;
    } gpu_times;
};
