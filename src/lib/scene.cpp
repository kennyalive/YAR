#include "std.h"
#include "common.h"
#include "scene.h"
#include "project.h"

#include "obj_loader.h"
#include "pbrt_loader.h"
#include "test_scenes.h"

static void finalize_scene(Scene& scene) {
    for (auto [i, light] : enumerate(scene.lights.diffuse_rectangular_lights)) {
        scene.geometries.triangle_meshes.emplace_back(light.get_geometry());
        
        Render_Object render_object;
        render_object.area_light = {Light_Type::diffuse_rectangular, (int)i};
        render_object.geometry = {Geometry_Type::triangle_mesh, (int)scene.geometries.triangle_meshes.size()-1};
        render_object.object_to_world_transform = Matrix3x4::identity;
        render_object.world_to_object_transform = Matrix3x4::identity;

        scene.render_objects.push_back(render_object);
    }
}

Scene load_scene(const YAR_Project& project) {
    Scene scene;

    if (project.scene_type == Scene_Type::pbrt) {
        scene = load_pbrt_project(project);
    }
    else if (project.scene_type == Scene_Type::obj) {
        scene = load_obj_project(project);
    }
    else {
        error("load_scene: unknown scene type");
    }

    if (project.has_camera_to_world)
        scene.view_points = {project.camera_to_world};

    finalize_scene(scene);

    ASSERT(scene.fovy != 0.f);
    return scene;
}

