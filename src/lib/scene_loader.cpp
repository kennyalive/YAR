#include "std.h"
#include "common.h"
#include "scene_loader.h"

#include "yar_project.h"

// defined in pbrt_scene.cpp
Scene load_pbrt_scene(const YAR_Project& project);
// defined in obj_scene.cpp
Scene load_obj_scene(const YAR_Project& project);
// defined in yar_project.cpp
YAR_Project parse_yar_file(const std::string& yar_file_path);

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

static void finalize_scene(Scene& scene) {
    for (auto [i, light] : enumerate(scene.lights.diffuse_rectangular_lights)) {
        scene.geometries.triangle_meshes.emplace_back(light.get_geometry());
        
        Scene_Object scene_object;
        scene_object.area_light = {Light_Type::diffuse_rectangular, (int)i};
        scene_object.geometry = {Geometry_Type::triangle_mesh, (int)scene.geometries.triangle_meshes.size()-1};
        scene_object.object_to_world_transform = Matrix3x4::identity;
        scene_object.world_to_object_transform = Matrix3x4::identity;

        scene.objects.push_back(scene_object);
    }
}

Scene load_scene(const std::string& input_file) {
    YAR_Project project = create_yar_project(input_file);

    Scene scene;
    if (project.scene_type == Scene_Type::pbrt) {
        scene = load_pbrt_scene(project);
    }
    else {
        ASSERT(project.scene_type == Scene_Type::obj);
        scene = load_obj_scene(project);
    }

    scene.type = project.scene_type;
    scene.path = project.scene_path.string();

    if (project.image_resolution != Vector2i{})
        scene.image_resolution = project.image_resolution;
    if (scene.image_resolution == Vector2i{})
        scene.image_resolution = Vector2i{ 1280, 720 };

    if (project.render_region != Bounds2i{})
        scene.render_region = project.render_region;
    if (scene.render_region == Bounds2i{})
        scene.render_region = Bounds2i{ {0, 0}, scene.image_resolution };

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