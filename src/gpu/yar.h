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

struct Global_Textures
{
    Vk_Image black_texture;

    void create();
    void destroy();
};

class YAR
{
public:
    void initialize(GLFWwindow* glfw_window, int gpu_index);
    void shutdown();
    void recreate_swapchain();

    bool vsync_enabled() const { return ui.vsync; }
    void toggle_ui() { ui.show_ui = !ui.show_ui; }

    void load_project(const std::string& input_file);
    void run_frame();

private:
    void release_resolution_dependent_resources();
    void restore_resolution_dependent_resources();
    void write_resolution_dependent_descriptors();

    void draw_frame();
    void draw_imgui();

    void start_reference_renderer();
    void do_run_reference_renderer(const Reference_Renderer_Config& reference_renderer_config, const Scene_Overrides& overrides);
    void wait_for_reference_renderer();

private:
    uint32_t frame_index = 0;
    uint32_t accumulation_index = 0;
    bool spp4 = false;

    Flying_Camera flying_camera;
    Global_Textures global_textures;
    Descriptor_Heap_Layout descriptor_heap_layout;
    Descriptor_Heap descriptor_heap;
    Kernels kernels;

    Vk_Time_Keeper time_keeper;
    Vk_Timer* timer_frame = nullptr;
    Vk_Timer* timer_ui = nullptr;

    std::atomic_bool reference_renderer_running = false;
    std::jthread reference_renderer_thread;

    Scene scene;
    GPU_Scene gpu_scene;
    UI ui;

    Path_Tracing_Renderer path_tracing_renderer;
    Direct_Lighting_Renderer direct_lighting_renderer;
};
