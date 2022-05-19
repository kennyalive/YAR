#include "std.h"
#include "lib/common.h"
#include "reference_renderer.h"
#include "test.h"

#include "getopt/getopt.h"

enum Options {
    OPT_HELP = 128, // start with some offset because getopt library reserves some values like '?' or '!'
    OPT_RUN_TESTS,
    OPT_THREAD_COUNT,
    OPT_RENDER_REGION_X,
    OPT_RENDER_REGION_Y,
    OPT_RENDER_REGION_W,
    OPT_RENDER_REGION_H,
    OPT_CROP_IMAGE_BY_RENDER_REGION,
    OPT_RNG_SEED_OFFSET,
    OPT_FLIP_HORIZONTALLY,
    OPT_FORCE_REBUILD_KDTREE_CACHE,
    OPT_OUTPUT_DIRECTORY,
    OPT_OUTPUT_FILENAME_SUFFIX,
    OPT_OPENEXR_DISABLE_VARYING_ATTRIBUTES,
    OPT_OPENEXR_COMPRESS,
    OPT_SAMPLES_PER_PIXEL,
    OPT_FILM_RESOLUTION,
    OPT_CHECKPOINT,
};

static const getopt_option_t option_list[] =
{
    { "help", 0, GETOPT_OPTION_TYPE_NO_ARG, nullptr, OPT_HELP,
        "print this help text" },

    { "test", 0, GETOPT_OPTION_TYPE_OPTIONAL, nullptr, OPT_RUN_TESTS,
        "run the test(s)", "test_name" },

    { "nthreads", 0, GETOPT_OPTION_TYPE_REQUIRED, 0, OPT_THREAD_COUNT,
        "specify thread count", "thread_count" },

    { "x", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_RENDER_REGION_X,
        "set render region top-left corner x coordinate", "x" },
    { "y", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_RENDER_REGION_Y,
        "set render region top-left corner y coordinate", "y" },
    { "w", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_RENDER_REGION_W,
        "set render region width", "width" },
    { "h", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_RENDER_REGION_H,
        "set render region height", "height" },

    { "crop", 0, GETOPT_OPTION_TYPE_NO_ARG, 0, OPT_CROP_IMAGE_BY_RENDER_REGION,
        "crop image by render region rectangle" },

    { "flip", 0, GETOPT_OPTION_TYPE_NO_ARG, nullptr, OPT_FLIP_HORIZONTALLY,
        "flip image horizontally" },

    { "seedoffset", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_RNG_SEED_OFFSET,
        "this value is added to per-pixel RNG seed", "integer_number" },

    { "force-rebuild-kdtree-cache", 0, GETOPT_OPTION_TYPE_NO_ARG, nullptr, OPT_FORCE_REBUILD_KDTREE_CACHE,
        "force rebuild of kdtree cache for current scene" },

    { "directory", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_OUTPUT_DIRECTORY,
        "location where to store output images", "directory_path" },

    { "suffix", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_OUTPUT_FILENAME_SUFFIX,
        "suffix that will be added to the output image filename", "string" },

    { "checkpoint", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_CHECKPOINT,
        "start or resume multi-session rendering", "checkpoint_directory_path" },

    { "openexr-disable-varying-attributes", 0, GETOPT_OPTION_TYPE_NO_ARG, nullptr, OPT_OPENEXR_DISABLE_VARYING_ATTRIBUTES,
        "do not generate OpenEXR custom attributes that vary between render sessions" },

    { "openexr-compress", 0, GETOPT_OPTION_TYPE_NO_ARG, nullptr, OPT_OPENEXR_COMPRESS,
        "enable OpenEXR zip compression of image data" },

    { "spp", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_SAMPLES_PER_PIXEL,
        "set samples per pixel value (overrides project settings)",
        "positive_integer_number" },

    { "resolution", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_FILM_RESOLUTION,
        "specify resolution of the output image (overrides project settings)",
        "640x480, 1080p, QHD, 4K, etc" },

    GETOPT_OPTIONS_END
};

static void print_help_string(getopt_context_t* ctx) {
    char buffer[4096];
    printf("Usage: RAY.exe <pbrt_file or yar_file> [options...]\n");
    printf("Options:\n%s\n", getopt_create_help_string(ctx, buffer, sizeof(buffer)));
}

static std::vector<std::string> read_list_file(const std::string& list_file)
{
    std::string content = read_text_file(list_file);
    std::vector<std::string> filenames;
    size_t start = SIZE_MAX;
    size_t last;
    for (size_t i = 0; i < content.size(); i++) {
        if (content[i] == '\n') { // handle new line
            if (start != SIZE_MAX) {
                filenames.emplace_back(content.data() + start, content.data() + last + 1);
                start = SIZE_MAX;
            }
        }
        else if (content[i] > 32) { // handle non-whitespace symbols
            if (start == SIZE_MAX) {
                start = i;
            }
            last = i;
        }
    }
    if (start != SIZE_MAX) { // handle last filename
        filenames.emplace_back(content.data() + start, content.data() + last + 1);
    }
    return filenames;
}

int main(int argc, char** argv) {
    initialize_fp_state();
    getopt_context_t ctx;
    if (getopt_create_context(&ctx, argc, (const char**)argv, option_list) < 0) {
        printf("internal error: invalid configuration of commnad line option list");
        return 1;
    }

    std::vector<std::string> files;
    Renderer_Options options;

    bool is_render_region_specified = false;
    Vector2i render_region_position {0, 0};
    // Default render region size is 1 pixel, so if you want to debug a single pixel then 
    // it's enough to specify only render region position (for example, --x 120 --y 253).
    Vector2i render_region_size { 1, 1 };

    int opt;
    while ((opt = getopt_next(&ctx)) != -1) {
        if (opt == '?') {
            printf("unknown flag: %s\n", ctx.current_opt_arg);
            print_help_string(&ctx);
            return 0;
        }
        else if (opt == '!') {
            printf("invalid flag usage: %s\n", ctx.current_opt_arg);
            print_help_string(&ctx);
            return 0;
        }
        else if (opt == '+') {
            std::string filename = ctx.current_opt_arg;
            if (to_lower(fs::path(filename).extension().string()) == ".list") {
                auto filenames = read_list_file(filename);
                files.insert(files.end(), filenames.begin(), filenames.end());
            }
            else {
                files.push_back(filename);
            }
        }
        else if (opt == OPT_HELP) {
            print_help_string(&ctx);
            return 0;
        }
        else if (opt == OPT_RUN_TESTS) {
            printf("running tests...\n");
            run_tests(ctx.current_opt_arg ? ctx.current_opt_arg : "");
            return 0;
        }
        else if (opt == OPT_THREAD_COUNT) {
            options.thread_count = atoi(ctx.current_opt_arg);
            if (options.thread_count < 0 || options.thread_count > 1024)
                options.thread_count = 0;
        }
        else if (opt == OPT_RENDER_REGION_X) {
            render_region_position.x = atoi(ctx.current_opt_arg);
            is_render_region_specified = true;
        }
        else if (opt == OPT_RENDER_REGION_Y) {
            render_region_position.y = atoi(ctx.current_opt_arg);
            is_render_region_specified = true;
        }
        else if (opt == OPT_RENDER_REGION_W) {
            render_region_size.x = std::max(1, atoi(ctx.current_opt_arg));
            is_render_region_specified = true;
        }
        else if (opt == OPT_RENDER_REGION_H) {
            render_region_size.y = std::max(1, atoi(ctx.current_opt_arg));
            is_render_region_specified = true;
        }
        else if (opt == OPT_CROP_IMAGE_BY_RENDER_REGION) {
            options.crop_image_by_render_region = true;
        }
        else if (opt == OPT_RNG_SEED_OFFSET) {
            options.rng_seed_offset = atoi(ctx.current_opt_arg);
        }
        else if (opt == OPT_FLIP_HORIZONTALLY) {
            options.flip_image_horizontally = true;
        }
        else if (opt == OPT_FORCE_REBUILD_KDTREE_CACHE) {
            options.force_rebuild_kdtree_cache = true;
        }
        else if (opt == OPT_OPENEXR_DISABLE_VARYING_ATTRIBUTES) {
            options.openexr_disable_varying_attributes = true;
        }
        else if (opt == OPT_OPENEXR_COMPRESS) {
            options.openexr_enable_compression = true;
        }
        else if (opt == OPT_OUTPUT_DIRECTORY) {
            options.output_directory = ctx.current_opt_arg;
        }
        else if (opt == OPT_OUTPUT_FILENAME_SUFFIX) {
            options.output_filename_suffix = ctx.current_opt_arg;
        }
        else if (opt == OPT_CHECKPOINT) {
            options.checkpoint_directory = ctx.current_opt_arg;
        }
        else if (opt == OPT_SAMPLES_PER_PIXEL) {
            options.samples_per_pixel = atoi(ctx.current_opt_arg);
            ASSERT(options.samples_per_pixel > 0);
        }
        else if (opt == OPT_FILM_RESOLUTION) {
            int width, height;
            if (sscanf(ctx.current_opt_arg, "%dx%d", &width, &height) == 2) {
                ASSERT(width > 0 && height > 0);
                options.film_resolution = Vector2i{ width, height };
            }
            else {
                std::string s = to_lower(ctx.current_opt_arg);
                if (s == "720p" || s == "hd") {
                    options.film_resolution = Vector2i{ 1280, 720 };
                }
                else if (s == "1080p" || s == "fhd") {
                    options.film_resolution = Vector2i{ 1920, 1080 };
                }
                else if (s == "1440p" || s == "qhd") {
                    options.film_resolution = Vector2i{ 2560, 1440 };
                }
                else if (s == "2160p" || s == "uhd" || s == "4k") {
                    options.film_resolution = Vector2i{ 3840, 2160 };
                }
                else {
                    printf("Unsupported argument for --resolution option: %s\n", ctx.current_opt_arg);
                    printf("Here are the examples of valid arguments: 1280x720, FHD, 1440p, 4K\n");
                    return 1;
                }
            }
        }
        else {
           ASSERT(!"unknown option");
        }
    }

    if (is_render_region_specified) {
        options.render_region.p0 = render_region_position;
        options.render_region.p1 = render_region_position + render_region_size;
    }

    if (files.empty()) {
        printf("input file(s) is not specified\n");
        print_help_string(&ctx);
        return 1;
    }
    for (const std::string& input_file : files) {
        cpu_renderer_render(input_file, options);
    }
    return 0;
}
