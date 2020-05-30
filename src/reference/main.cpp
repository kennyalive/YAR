#include "std.h"
#include "lib/common.h"
#include "reference_renderer.h"
#include "test.h"

#include "getopt/getopt.h"

static const getopt_option_t option_list[] =
{
    { "help", 0, GETOPT_OPTION_TYPE_NO_ARG, 0, 'h', "print this help text", 0 },
    { "test", 0, GETOPT_OPTION_TYPE_NO_ARG, 0, 't', "run the tests", 0 },
    { "nthreads", 0, GETOPT_OPTION_TYPE_REQUIRED, 0, 'n', "specify thread count", "thread_count" },
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
        else if (opt == 'h') {
            print_help_string(&ctx);
            return 0;
        }
        else if (opt == 't') {
            printf("running tests...\n");
            run_tests();
            return 0;
        }
        else if (opt == 'n') {
            options.thread_count = atoi(ctx.current_opt_arg);
            if (options.thread_count < 0 || options.thread_count > 1024)
                options.thread_count = 0;
        }
        else if (opt == '+') {
            files.push_back(ctx.current_opt_arg);
        } else {
           ASSERT(!"unknown option");
        }
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
