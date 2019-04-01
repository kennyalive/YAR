#include "std.h"
#include "reference_renderer.h"
#include "lib/io.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: yar <yar_file>\n");
        return 1;
    }
    YAR_Project project = parse_project(argv[1]);
    Scene_Data scene_data = load_scene(project.scene_type, project.scene_path);

    Bounds2i render_region = project.render_region;
    if (render_region == Bounds2i{})
        render_region = { {0, 0}, project.image_resolution }; // default render region

    Reference_Renderer_Input input{};
    input.scene_data = &scene_data;
    input.image_resolution = project.image_resolution;
    input.render_region = render_region;
    input.camera_to_world = project.camera_to_world;
    render_reference_image(input);
    return 0;
}
