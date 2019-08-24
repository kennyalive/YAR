#include "std.h"
#include "common.h"
#include "scene.h"
#include "project.h"

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

Scene create_scene(const YAR_Project& project) {
    Scene scene;
    if (project.scene_type == Scene_Type::pbrt) {
        scene = load_pbrt_project(project);
    }
    else if (project.scene_type == Scene_Type::test) {
        if (project.scene_path == "conference")
            scene = load_conference_scene();
        else if (project.scene_path == "bunny")
            scene = load_bunny_scene();
        else if (project.scene_path == "buddha")
            scene = load_buddha_scene();
        else if (project.scene_path == "hairball")
            scene = load_hairball_scene();
        else if (project.scene_path == "mori_knob")
            scene = load_mori_knob();
    }
    else {
        error("load_scene: unknown scene type");
    }
    finalize_scene(scene);
    return scene;
}

