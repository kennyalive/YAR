#include "std.h"
#include "reference_renderer.h"
#include "lib/io.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: yar <yar_file>\n");
        return 1;
    }
    YAR_File yar_file = load_yar_file(argv[1]);
    Scene_Data scene_data = load_scene(yar_file.scene_type, yar_file.scene_path);

    Bounds2i render_region = yar_file.render_region;
    if (render_region == Bounds2i{})
        render_region = { {0, 0}, yar_file.image_resolution }; // default render region

    Reference_Renderer_Input input{};
    input.scene_data = &scene_data;
    input.image_resolution = yar_file.image_resolution;
    input.render_region = render_region;
    input.camera_to_world = yar_file.camera_to_world;
    render_reference_image(input);
    return 0;
}
