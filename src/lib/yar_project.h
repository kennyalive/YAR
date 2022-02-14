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

    // Location of the scene file (*.pbrt, *.obj, etc).
    // This path should be directly accessible by c/c++ file API -
    // it must be either an absolute path or to be relative to the program's current working directory.
    fs::path scene_path;

    Vector2i film_resolution;
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

    // The lights defined in yar project file. Another source of lights are the lights
    // defined in specific scene formats, for example, pbrt scene. The lights from the
    // yar project are merged with the scene's native lights in the final Scene object.
    std::vector<Point_Light> point_lights;
    std::vector<Directional_Light> directional_lights;
    std::vector<Diffuse_Rectangular_Light> diffuse_rectangular_lights;

    std::vector<YAR_Instance> instances;
    std::vector<std::string> ignore_geometry_names;

    YAR_Obj_Info obj_info;
};

YAR_Project parse_yar_file(const std::string& yar_file_path);
