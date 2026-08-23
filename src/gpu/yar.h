#pragma once

#include "descriptor_heap.h"
#include "descriptor_heap_layout.h"
#include "gpu_scene.h"
#include "kernels.h"
#include "vk.h"
#include "ui.h"

#include "renderers/direct_lighting_renderer.h"
#include "renderers/path_tracing_renderer.h"

#include "lib/flying_camera.h"
#include "lib/matrix.h"
#include "lib/scene.h"

struct GLFWwindow;
struct Reference_Renderer_Config;
struct Scene_Overrides;

class YAR
{
public:
    void initialize(GLFWwindow* glfw_window, int gpu_index);
    void shutdown();
    void recreate_swapchain();

    bool vsync_enabled() const { return ui.vsync; }
    void toggle_ui() { ui.show_ui = !ui.show_ui; }

    void load_project(const std::string& input_file);
    void unload_project();

    void run_frame();

private:
    void create_resolution_dependent_resources();
    void destroy_resolution_dependent_resources();

    void write_resolution_dependent_descriptors();
    void write_common_descriptors();

    void draw_frame();
    void draw_imgui();

    void start_reference_renderer();
    void do_run_reference_renderer(const Reference_Renderer_Config& reference_renderer_config, const Scene_Overrides& overrides);
    void wait_for_reference_renderer();

private:
    Descriptor_Heap_Layout descriptor_heap_layout;
    Descriptor_Heap descriptor_heap;
    Kernels kernels;
    Vk_Image none_texture;

    Scene scene;
    GPU_Scene gpu_scene;

    UI ui;
    Flying_Camera flying_camera;
    double last_frame_time = 0.0;

    Vk_Time_Keeper time_keeper;
    Vk_Timer* timer_frame = nullptr;
    Vk_Timer* timer_ui = nullptr;

    uint32_t frame_index = 0;

    Path_Tracing_Renderer path_tracing_renderer;
    Direct_Lighting_Renderer direct_lighting_renderer;

    std::atomic_bool reference_renderer_running = false;
    std::jthread reference_renderer_thread;
};
