#include "std.h"
#include "lib/common.h"
#include "reference_renderer.h"
#include "scene_context.h"
#include "test.h"

#include "lib/scene_loader.h"

#include "getopt/getopt.h"

constexpr int time_category_field_width = 21; // for printf 'width' specifier

struct Command_Line_Options {
    int thread_count = 0;

    Bounds2i render_region;
    bool crop_image_by_render_region = true;

    // Can be useful during debugging to vary random numbers and get configuration that
    // reproduces desired behavior.
    int rng_seed_offset = 0;

    // Can be used to match output of the renderer that uses left-handed coordinate system.
    bool flip_image_horizontally = false;

    bool force_rebuild_kdtree_cache = false;

    // This option disables generation of openexr custom attributes that vary between render sessions.
    // Examples of varying attributes: render time, variance.
    // Examples of non-varying attributes: output file name, per pixel sample count.
    bool openexr_disable_varying_attributes = false;

    // Enables OpenEXR feature to store image data in compressed form (zip)
    bool openexr_enable_compression = false;

    std::string output_directory;
    std::string output_filename_suffix;
    std::string checkpoint_directory;

    int samples_per_pixel = 0; // overrides project settings
    Vector2i film_resolution; // overrides project settings

    bool override_rendering_algorithm = false;
    Raytracer_Config::Rendering_Algorithm rendering_algorithm;
};

struct Parsed_Command_Line {
    std::vector<std::string> files;
    Command_Line_Options options;
    bool exit_app = false;
    int exit_code = 0;

    Parsed_Command_Line() = default;
    Parsed_Command_Line(int exit_code) : exit_app(true), exit_code(exit_code) {}
};

enum Options {
    OPT_HELP = 128, // start with some offset because getopt library reserves some values like '?' or '!'
    OPT_RUN_TESTS,
    OPT_THREAD_COUNT,
    OPT_RENDER_REGION_X,
    OPT_RENDER_REGION_Y,
    OPT_RENDER_REGION_W,
    OPT_RENDER_REGION_H,
    OPT_DO_NOT_CROP_IMAGE_BY_RENDER_REGION,
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
    OPT_PATH_TRACING,
    OPT_DIRECT_LIGHTING,
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

    { "nocrop", 0, GETOPT_OPTION_TYPE_NO_ARG, 0, OPT_DO_NOT_CROP_IMAGE_BY_RENDER_REGION,
        "do not crop image by render region rectangle" },

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

    { "path", 0, GETOPT_OPTION_TYPE_NO_ARG, nullptr, OPT_PATH_TRACING,
        "force path tracing rendering algorithm" },

    { "direct", 0, GETOPT_OPTION_TYPE_NO_ARG, nullptr, OPT_DIRECT_LIGHTING,
        "force direct lighting rendering algorithm" },

    GETOPT_OPTIONS_END
};

static void print_help_string(getopt_context_t* ctx)
{
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

static Parsed_Command_Line parse_command_line(int argc, char** argv)
{
    getopt_context_t ctx;
    if (getopt_create_context(&ctx, argc, (const char**)argv, option_list) < 0) {
        printf("internal error: invalid configuration of commnad line option list");
        return 1;
    }

    std::vector<std::string> files;
    Command_Line_Options options;

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
        else if (opt == OPT_DO_NOT_CROP_IMAGE_BY_RENDER_REGION) {
            options.crop_image_by_render_region = false;
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
        else if (opt == OPT_PATH_TRACING) {
            options.override_rendering_algorithm = true;
            options.rendering_algorithm = Raytracer_Config::Rendering_Algorithm::path_tracer;
        }
        else if (opt == OPT_DIRECT_LIGHTING) {
            options.override_rendering_algorithm = true;
            options.rendering_algorithm = Raytracer_Config::Rendering_Algorithm::direct_lighting;
        }
        else {
            ASSERT(!"unknown option");
        }
    }

    if (files.empty()) {
        printf("input file(s) is not specified\n");
        print_help_string(&ctx);
        return 1;
    }
    if (is_render_region_specified) {
        options.render_region.p0 = render_region_position;
        options.render_region.p1 = render_region_position + render_region_size;
    }
    else if (options.film_resolution != Vector2i{}) {
        // Custom film resolution invalidates native scene's render region
        // Set render region to match custom film resolution.
        options.render_region = Bounds2i{ {0, 0}, options.film_resolution };
    }
    Parsed_Command_Line cmdline;
    cmdline.files = files;
    cmdline.options = options;
    return cmdline;
}

static void process_input_file(const std::string& input_file, const Command_Line_Options& options)
{
    Timestamp t_start;
    printf("Loading: %s\n", input_file.c_str());

    //
    // Load scene.
    //
    Timestamp t_project;
    Scene scene = load_scene(input_file);
    float project_load_time = elapsed_seconds(t_project);
    printf("%-*s %.3f seconds\n", time_category_field_width, "Parse project", project_load_time);

    //
    // Override requested parts of the scene.
    //
    if (options.film_resolution != Vector2i{}) {
        scene.film_resolution = options.film_resolution;
    }
    if (options.render_region != Bounds2i{}) {
        scene.render_region = options.render_region;
    }
    if (options.samples_per_pixel > 0) {
        int k = (int)std::ceil(std::sqrt(options.samples_per_pixel));
        scene.raytracer_config.x_pixel_sample_count = k;
        scene.raytracer_config.y_pixel_sample_count = k;
    }
    if (options.override_rendering_algorithm) {
        scene.raytracer_config.rendering_algorithm = options.rendering_algorithm;
    }
    int thread_count = options.thread_count;
    if (!thread_count) {
        thread_count = std::max(1u, std::thread::hardware_concurrency());
    }

    //
    // Render scene.
    //
    Renderer_Configuration config;
    config.thread_count = thread_count;
    config.checkpoint_directory = options.checkpoint_directory;
    config.rebuild_kdtree_cache = options.force_rebuild_kdtree_cache;
    config.rng_seed_offset = options.rng_seed_offset;

    Scene_Context scene_ctx;
    init_scene_context(scene, config, scene_ctx);
    float load_time = elapsed_seconds(t_start);
    printf("%-*s %.3f seconds\n\n", time_category_field_width, "Total loading time", load_time);

    double variance_estimate = 0.0;
    float render_time = 0.f;
    Image image = render_scene(scene_ctx, &variance_estimate, &render_time);

    printf("%-*s %.3f seconds\n", 12, "Render time", render_time);
    printf("%-*s %.6f\n", 12, "Variance", variance_estimate);
    printf("%-*s %.6f\n", 12, "StdDev", std::sqrt(variance_estimate));

    //
    // Save image
    //
    // --nocrop
    if (!options.crop_image_by_render_region) {
        ASSERT(Vector2i(image.width, image.height) == scene.render_region.size());
        if (scene.render_region != Bounds2i{ {0, 0}, scene.film_resolution }) {
            image.extend_to_region(scene.film_resolution, scene.render_region.p0);
        }
    }
    // --flip
    if (options.flip_image_horizontally) {
        image.flip_horizontally();
    }

    std::string image_filename;
    if (!scene.output_filename.empty()) {
        image_filename = fs::path(scene.output_filename).replace_extension().string();
    }
    else {
        image_filename = fs::path(input_file).stem().string();
    }
    if (!options.output_directory.empty()) {
        image_filename = (fs::path(options.output_directory) / fs::path(image_filename)).string();
    }
    image_filename += options.output_filename_suffix;
    image_filename += ".exr"; // output is OpenEXR image

    EXR_Write_Params write_params;
    write_params.custom_attributes = EXR_Custom_Attributes {
        .input_file = input_file,
        .spp = scene_ctx.pixel_sampler_config.get_samples_per_pixel(),
        .variance = (float)variance_estimate,
        .load_time = load_time,
        .render_time = render_time,
    };

    if (!write_openexr_image(image_filename, image, write_params)) {
        error("Failed to save rendered image: %s", image_filename.c_str());
    }
    printf("Saved output image to %s\n\n", image_filename.c_str());
}

int main(int argc, char** argv)
{
    initialize_fp_state();
    const Parsed_Command_Line cmdline = parse_command_line(argc, argv);
    if (cmdline.exit_app) {
        return cmdline.exit_code;
    }

    for (const std::string& input_file : cmdline.files) {
        process_input_file(input_file, cmdline.options);
    }
    return 0;
}
