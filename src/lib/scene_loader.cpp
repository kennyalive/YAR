#include "std.h"
#include "common.h"
#include "path.h"
#include "scene_loader.h"

#include "stb/stb_image.h"

// defined in pbrt_scene.cpp
void load_pbrt_scene(Scene& scene);

static void finalize_scene(Scene& scene) {
    for (Scene_Object& scene_object : scene.objects) {
        scene_object.object_to_world_normal_transform = Matrix3x4::zero;
        for (int i = 0; i < 3; i++)
            for (int k = 0; k < 3; k++)
                scene_object.object_to_world_normal_transform.a[i][k] = scene_object.world_to_object_transform.a[k][i];
    }

    // Add default light if no other light is specified.
    if (!scene.lights.has_lights()) {
        Directional_Light light;
        light.direction = Vector3(1, 1, 1).normalized();
        light.irradiance = ColorRGB(5, 5, 5);
        scene.lights.directional_lights.push_back(light);
    }
    scene.lights.update_total_light_count();
}

Scene load_scene(const String& input_file) {
    String_View extension = path_extension(input_file);
    if (extension.size == 0)
        fatal("Unknown file type because there is no extension: %s. The supported file type is: pbrt", input_file.data());
    if (!equals_ignore_case(extension, "pbrt"))
        fatal("Unsupported file extension: %.*s", (int)extension.size, extension.data);

    Scene scene;
    scene.type = Scene_Type::pbrt;
    scene.path = input_file;

    load_pbrt_scene(scene);

    // In pbrt texture coordinate space has(0, 0) at the lower left corner.
    // Workaround with flipping texture coordinates instead is not robust
    // enough because it doesn't handle procedural texturing case.
    stbi_set_flip_vertically_on_load(true);

    if (scene.film_resolution == Vector2i{})
        scene.film_resolution = Vector2i{ 1920, 1080 };

    if (scene.render_region == Bounds2i{})
        scene.render_region = Bounds2i{ {0, 0}, scene.film_resolution };

    if (scene.view_points.empty())
        scene.view_points = { Matrix3x4::identity };

    if (!scene.camera_fov_y)
        scene.camera_fov_y = 45.f;

    finalize_scene(scene);

    ASSERT(scene.film_resolution != Vector2i{});

    // check that render region is within the film dimensions
    ASSERT(scene.render_region.p0 >= Vector2i{});
    ASSERT(scene.render_region.p0 < scene.render_region.p1);
    ASSERT(scene.render_region.p1 <= scene.film_resolution);

    return scene;
}

int add_scene_texture(const Texture_Descriptor& texture_desc, Scene* scene)
{
    for (size_t i = 0; i < scene->texture_descriptors.size(); i++) {
        if (scene->texture_descriptors[i] == texture_desc)
            return (int)i;
    }
    scene->texture_descriptors.push_back(texture_desc);
    return (int)scene->texture_descriptors.size() - 1;
}

int add_scene_texture(const String& file_name, Scene* scene)
{
    ASSERT(!file_name.empty());
    return add_scene_texture(Texture_Descriptor{ .file_name = file_name }, scene);
}

int add_scene_material_parameter(const Parameter& parameter, Scene* scene)
{
    scene->material_parameters.push_back(parameter);
    return (int)scene->material_parameters.size() - 1;
}
