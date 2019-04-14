#include "std.h"
#include "lib/common.h"
#include "reference_renderer.h"
#include "lib/io.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: yar <yar_file>\n");
        return 1;
    }
    YAR_Project project = parse_project(argv[1]);
    Renderer_Options options;
    render_reference_image(project, options);
    return 0;
}
