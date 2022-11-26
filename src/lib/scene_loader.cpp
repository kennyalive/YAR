#include "std.h"
#include "common.h"
#include "scene_loader.h"

#include "yar_project.h"

#include "stb/stb_image.h"

// defined in pbrt_scene.cpp
Scene load_pbrt_scene(const YAR_Project& project);
// defined in obj_scene.cpp
Scene load_obj_scene(const YAR_Project& project);


static YAR_Project create_yar_project(const std::string& input_file) {
    fs::path path(input_file);
    if (!path.has_extension())
        error("Unknown file type because there is no extension: %s. The supported file types are: yar, pbrt, obj", input_file.c_str());

    YAR_Project project;
    std::string ext = to_lower(path.extension().string());
    if (ext == ".yar") {
        project = parse_yar_file(input_file);
    }
    else if (ext == ".pbrt") {
        project.scene_type = Scene_Type::pbrt;
        project.scene_path = input_file;
    }
    else {
        error("Unsupported file extension: %s", ext.c_str());
        return YAR_Project{};
    }
    return project;
}

static void add_light_sources_from_yar_project(Scene& scene, const YAR_Project& project) {
    scene.lights.point_lights.insert(scene.lights.point_lights.end(),
        project.point_lights.begin(), project.point_lights.end());

    scene.lights.directional_lights.insert(scene.lights.directional_lights.end(),
        project.directional_lights.begin(), project.directional_lights.end());

    for (const Diffuse_Rectangular_Light& light : project.diffuse_rectangular_lights) {
        scene.lights.diffuse_rectangular_lights.push_back(light);
        scene.geometries.triangle_meshes.emplace_back(light.get_geometry());

        Scene_Object scene_object;
        scene_object.area_light = {Light_Type::diffuse_rectangular, (int)scene.lights.diffuse_rectangular_lights.size()-1};
        scene_object.geometry = {Geometry_Type::triangle_mesh, (int)scene.geometries.triangle_meshes.size()-1};
        scene_object.object_to_world_transform = Matrix3x4::identity;
        scene_object.world_to_object_transform = Matrix3x4::identity;
        scene.objects.push_back(scene_object);
    }
}

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

Scene load_scene(const std::string& input_file) {
    YAR_Project project = create_yar_project(input_file);

    Scene scene;
    if (project.scene_type == Scene_Type::pbrt) {
        scene = load_pbrt_scene(project);

        // In pbrt texture coordinate space has(0, 0) at the lower left corner.
        // Workaround with flipping texture coordinates instead is not robust
        // enough because it doesn't handle procedural texturing case.
        stbi_set_flip_vertically_on_load(true);
    }
    else {
        ASSERT(project.scene_type == Scene_Type::obj);
        scene = load_obj_scene(project);
    }

    add_light_sources_from_yar_project(scene, project);

    scene.type = project.scene_type;
    scene.path = project.scene_path.string();

    if (project.film_resolution != Vector2i{})
        scene.film_resolution = project.film_resolution;
    if (scene.film_resolution == Vector2i{})
        scene.film_resolution = Vector2i{ 1920, 1080 };

    if (project.render_region != Bounds2i{})
        scene.render_region = project.render_region;
    if (scene.render_region == Bounds2i{})
        scene.render_region = Bounds2i{ {0, 0}, scene.film_resolution };

    if (project.obj_info.z_is_up_specified) {
        ASSERT(project.scene_type == Scene_Type::obj);
        scene.z_is_up = project.obj_info.z_is_up;
    }

    scene.mesh_disable_backfacing_culling = project.mesh_disable_backfacing_culling;

    if (!project.camera_to_world.is_zero())
        scene.view_points = { project.camera_to_world };
    if (scene.view_points.empty())
        scene.view_points = { Matrix3x4::identity };

    if (project.camera_fov_y)
        scene.camera_fov_y = project.camera_fov_y;
    if (!scene.camera_fov_y)
        scene.camera_fov_y = 45.f;

    finalize_scene(scene);
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

int add_scene_texture(const std::string& file_name, Scene* scene)
{
    ASSERT(!file_name.empty());
    return add_scene_texture(Texture_Descriptor{ .file_name = file_name }, scene);
}
