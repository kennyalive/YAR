#pragma once

#include "bounding_box.h"
#include "matrix.h"
#include "scene.h"
#include "vector.h"

// OBJ scene specific information.
struct YAR_Obj_Info {
    bool z_is_up_specified = false;
    bool z_is_up = false; // true - Z is up, false - Y is up

    bool left_handed_specified = false;
    bool left_handed = false;
};

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
    float camera_fov_y = 0.f;

    bool mesh_disable_backfacing_culling = false;
    bool mesh_invert_winding_order = false;

    // If the following option is enabled and the angle between face normals is larger or equal
    // to the crease angle then face normals will be used as vertex shading normals.
    bool mesh_use_crease_angle = false;
    float mesh_crease_angle = 0.f;

    // Lights defined in yar project file. The other source of lights is the
    // scene itself, for example, pbrt scene usually defines the lights. The
    // lights from yar project are merged with the native scene's lights in
    // the final Scene object.
    Lights lights;

    std::vector<YAR_Instance> instances;
    std::vector<std::string> ignore_geometry_names;

    YAR_Obj_Info obj_info;
};

YAR_Project parse_yar_file(const std::string& yar_file_path);
