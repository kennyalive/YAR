#include "std.h"
#include "lib/common.h"
#include "renderer.h"

#include "glfw/glfw3.h"
#include "imgui/imgui.h"

static std::string input_file;
static bool enable_vulkan_validation = false;
static int gpu_index = -1;

static int window_width = 960;
static int window_height = 720;

static GLFWwindow* window;

static bool parse_command_line(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--gpu") == 0) {
            if (i == argc - 1) {
                printf("--gpu value is missing\n");
            }
            else {
                gpu_index = std::stoi(argv[i + 1]);
                i++;
            }
        }
        else if (strcmp(argv[i], "--validation") == 0) {
            enable_vulkan_validation = true;
        }
        else if (strcmp(argv[i], "--data-dir") == 0) {
            if (i == argc-1) {
                printf("-data-dir value is missing\n");
            }
            else {
                extern std::string g_data_dir;
                g_data_dir = argv[i+1];
                i++;
            }
        }
        else if (strcmp(argv[i], "-help") == 0) {
            extern std::string g_data_dir;
            printf("%-25s Path to the data directory. Default: %s\n", "-data-dir", g_data_dir.c_str());
            printf("%-25s Enables Vulkan validation layers.\n", "-validation-layers");
            printf("%-25s Allows to assign debug names to Vulkan objects.\n", "-debug-names");
            printf("%-25s Shows this information.\n", "-help");
            return false;
        }
        else {
            input_file = argv[i];
        }
    }
    return true;
}

static void glfw_error_callback(int error, const char* description) {
    fprintf(stderr, "GLFW error: %s\n", description);
}

static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_ESCAPE) {
            glfwSetWindowShouldClose(window, GLFW_TRUE);
        } else if (key == GLFW_KEY_F11 || key == GLFW_KEY_ENTER && mods == GLFW_MOD_ALT) {
            static int last_window_xpos, last_window_ypos;
            static int last_window_width, last_window_height;

            VK_CHECK(vkDeviceWaitIdle(vk.device));
            GLFWmonitor* monitor = glfwGetWindowMonitor(window);
            if (monitor == nullptr) {
                glfwGetWindowPos(window, &last_window_xpos, &last_window_ypos);
                last_window_width = window_width;
                last_window_height = window_height;

                monitor = glfwGetPrimaryMonitor();
                const GLFWvidmode* mode = glfwGetVideoMode(monitor);
                glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            } else {
                glfwSetWindowMonitor(window, nullptr, last_window_xpos, last_window_ypos, last_window_width, last_window_height, 0);
            }
        } else if (key == GLFW_KEY_F10) {
            Renderer* renderer = (Renderer*)glfwGetWindowUserPointer(window);
            renderer->toggle_ui();
        }
    }
}

int run_realtime_renderer(bool enable_vulkan_validation, int gpu_index) {
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        error("glfwInit failed");

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    window = glfwCreateWindow(window_width, window_height, "YAR", nullptr, nullptr);
    ASSERT(window != nullptr);
    glfwSetKeyCallback(window, glfw_key_callback);

    Renderer renderer{};
    renderer.initialize(window, enable_vulkan_validation, gpu_index);
    glfwSetWindowUserPointer(window, &renderer);

    if (!input_file.empty())
        renderer.load_project(input_file);

    bool prev_vsync = renderer.vsync_enabled();
    bool window_active = true;

    while (!glfwWindowShouldClose(window)) {
        if (window_active)
            renderer.run_frame();

        glfwPollEvents();

        int width, height;
        glfwGetWindowSize(window, &width, &height);

        bool recreate_swapchain = false;
        if (prev_vsync != renderer.vsync_enabled()) {
            prev_vsync = renderer.vsync_enabled();
            recreate_swapchain = true;
        } else if (width != window_width || height != window_height) {
            window_width = width;
            window_height = height;
            recreate_swapchain = true;
        }

        window_active = (width != 0 && height != 0);
        if (!window_active)
            continue; 

        if (recreate_swapchain) {
            VK_CHECK(vkDeviceWaitIdle(vk.device));
            renderer.release_resolution_dependent_resources();
            vk_destroy_swapchain();
            vk_create_swapchain(renderer.vsync_enabled());
            renderer.restore_resolution_dependent_resources();
            recreate_swapchain = false;
        }
    }

    renderer.shutdown();
    glfwTerminate();
    return 0;
}

int main(int argc, char** argv) {
    if (!parse_command_line(argc, argv))
        return 0;

    run_realtime_renderer(enable_vulkan_validation, gpu_index);
    return 0;
}
