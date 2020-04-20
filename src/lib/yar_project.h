#pragma once

#include "bounding_box.h"
#include "matrix.h"
#include "scene.h"
#include "vector.h"

struct YAR_Instance {
    std::string geometry_name;
    Matrix3x4 transform;
};

struct YAR_Project {
    Scene_Type scene_type;
    fs::path scene_path;

    Vector2i image_resolution;
    Bounds2i render_region;
    Matrix3x4 camera_to_world = Matrix3x4{};

    float world_scale = 1.f;
    float camera_fov_y = 45.f;

    bool mesh_disable_backfacing_culling = false;
    bool mesh_invert_winding_order = false;
    float mesh_crease_angle = 0.f;

    // Lights defined in yar project file. The other source of lights is the
    // scene itself, for example, pbrt scene usually defines the lights. The
    // lights from yar project are merged with the native scene's lights in
    // the final Scene object.
    Lights lights;

    std::vector<YAR_Instance> instances;
    std::vector<std::string> ignore_geometry_names;
};
