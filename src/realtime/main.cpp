#include "std.h"
#include "lib/common.h"
#include "realtime_renderer.h"

#include "lib/platform.h"

#include "glfw/glfw3.h"
#include "imgui/imgui.h"

static std::string yar_project_file;
static bool enable_validation_layers = false;
static bool use_debug_names = false;

static int window_width = 960;
static int window_height = 720;

static bool parse_command_line(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--validation-layers") == 0) {
            enable_validation_layers = true;
        }
        else if (strcmp(argv[i], "--debug-names") == 0) {
            use_debug_names = true;
        }
        else if (strcmp(argv[i], "--data-dir") == 0) {
            if (i == argc-1) {
                printf("--data-dir value is missing\n");
            }
            else {
                extern std::string g_data_dir;
                g_data_dir = argv[i+1];
                i++;
            }
        }
        else if (strcmp(argv[i], "--help") == 0) {
            printf("%-25s Path to the data directory. Default is ./data.\n", "--data-dir");
            printf("%-25s Enables Vulkan validation layers.\n", "--validation-layers");
            printf("%-25s Allows to assign debug names to Vulkan objects.\n", "--debug-names");
            printf("%-25s Shows this information.\n", "--help");
            return false;
        }
        else {
            yar_project_file = argv[i];
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
        }
    }
}

int run_realtime_renderer(bool enable_validation_layers, bool use_debug_names) {
    Vk_Create_Info vk_create_info{};
    vk_create_info.enable_validation_layers = enable_validation_layers;
    vk_create_info.use_debug_names = use_debug_names;

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        error("glfwInit failed");

    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
    GLFWwindow* window = glfwCreateWindow(window_width, window_height, "YAR", nullptr, nullptr);
    ASSERT(window != nullptr);
    glfwSetKeyCallback(window, glfw_key_callback);

    Realtime_Renderer renderer{};
    renderer.initialize(vk_create_info, window);

    if (!yar_project_file.empty())
        renderer.load_project(yar_project_file);

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
            vk_release_resolution_dependent_resources();
            vk_restore_resolution_dependent_resources(renderer.vsync_enabled());
            renderer.restore_resolution_dependent_resources();
            recreate_swapchain = false;
        }
        platform::sleep(1);
    }

    renderer.shutdown();
    glfwTerminate();
    return 0;
}

int main(int argc, char** argv) {
    if (!parse_command_line(argc, argv))
        return 0;

    run_realtime_renderer(enable_validation_layers, use_debug_names);
    return 0;
}
