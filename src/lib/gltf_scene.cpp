#include "common.h"
#include "scene.h"

#define CGLTF_IMPLEMENTATION
#include "cgltf/cgltf.h"

void load_gltf_scene(Scene& scene)
{
    cgltf_options options{};
    cgltf_data* data = nullptr;
    cgltf_result result = cgltf_parse_file(&options, scene.path.data(), &data);
    if (result != cgltf_result_success) {
        error("Failed to parse glTF file %s (cgltf error %d)", scene.path.data(), result);
    }

    result = cgltf_load_buffers(&options, data, scene.path.data());
    if (result != cgltf_result_success) {
        cgltf_free(data);
        error("Failed to load glTF buffers for %s (cgltf error %d)", scene.path.data(), result);
    }

    result = cgltf_validate(data);
    if (result != cgltf_result_success) {
        cgltf_free(data);
        error("Invalid glTF file %s (cgltf error %d)", scene.path.data(), result);
    }

    printf("Loaded glTF (meshes: %zu, materials: %zu, cameras: %zu)\n", data->meshes_count, data->materials_count, data->cameras_count);
    cgltf_free(data);
}
