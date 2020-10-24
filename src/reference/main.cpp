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
};

static const getopt_option_t option_list[] =
{
    { "help", 0, GETOPT_OPTION_TYPE_NO_ARG, 0, OPT_HELP, "print this help text", 0 },
    { "test", 0, GETOPT_OPTION_TYPE_NO_ARG, 0, OPT_RUN_TESTS, "run the tests", 0 },
    { "nthreads", 0, GETOPT_OPTION_TYPE_REQUIRED, 0, OPT_THREAD_COUNT, "specify thread count", "thread_count" },
    { "x", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_RENDER_REGION_X, "render region top-left corner x coordinate", "x"},
    { "y", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_RENDER_REGION_Y, "render region top-left corner y coordinate", "y"},
    { "w", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_RENDER_REGION_W, "render region width", "width"},
    { "h", 0, GETOPT_OPTION_TYPE_REQUIRED, nullptr, OPT_RENDER_REGION_H, "render region height", "height"},
    GETOPT_OPTIONS_END
};

static void print_help_string(getopt_context_t* ctx) {
    printf("Usage:\n");
    printf("yar <yar_file> [options...]\n");

    char buffer[2048];
    printf("Options:\n");
    printf("%s\n", getopt_create_help_string(ctx, buffer, sizeof(buffer)));
}

int main(int argc, char** argv) {
    getopt_context_t ctx;
    if (getopt_create_context(&ctx, argc, (const char**)argv, option_list) < 0) {
        printf("internal error: invalid configuration of commnad line option list");
        return 1;
    }

    std::vector<std::string> files;
    Renderer_Options options;

    Vector2i render_region_position {-1, -1};
    Vector2i render_region_size;

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
            files.push_back(ctx.current_opt_arg);
        }
        else if (opt == OPT_HELP) {
            print_help_string(&ctx);
            return 0;
        }
        else if (opt == OPT_RUN_TESTS) {
            printf("running tests...\n");
            run_tests();
            return 0;
        }
        else if (opt == OPT_THREAD_COUNT) {
            options.thread_count = atoi(ctx.current_opt_arg);
            if (options.thread_count < 0 || options.thread_count > 1024)
                options.thread_count = 0;
        }
        else if (opt == OPT_RENDER_REGION_X) {
            render_region_position.x = atoi(ctx.current_opt_arg);
        }
        else if (opt == OPT_RENDER_REGION_Y) {
            render_region_position.y = atoi(ctx.current_opt_arg);
        }
        else if (opt == OPT_RENDER_REGION_W) {
            render_region_size.x = atoi(ctx.current_opt_arg);
        }
        else if (opt == OPT_RENDER_REGION_H) {
            render_region_size.y = atoi(ctx.current_opt_arg);
        }
        else {
           ASSERT(!"unknown option");
        }
    }

    if (render_region_position.x >= 0 &&
        render_region_position.y >= 0 &&
        render_region_size.x > 0 &&
        render_region_size.y > 0)
    {
        options.render_region.p0 = render_region_position;
        options.render_region.p1 = render_region_position + render_region_size;
    }

    if (files.empty()) {
        printf("input file(s) is not specified\n");
        print_help_string(&ctx);
        return 1;
    }
    for (const std::string& input_file : files) {
        render_reference_image(input_file, options);
    }
    return 0;
}
