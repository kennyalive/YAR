#include "realtime/demo.h"

#include <cstdio>
#include <cstring>
#include <string>

static bool enable_validation_layers = false;
static bool use_debug_names = false;

static bool parse_command_line(int argc, char** argv) {
    bool found_unknown_option = false;
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
            } else {
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
        else
            found_unknown_option = true;
    }
    if (found_unknown_option)
        printf("Use --help to list all options.\n");
    return true;
}

int run_vk_demo(bool enable_validation_layers, bool use_debug_names);

int main(int argc, char** argv) {
    if (!parse_command_line(argc, argv))
        return 0;

    run_vk_demo(enable_validation_layers, use_debug_names);
    return 0;
}
