#pragma once

#include "bounding_box.h"
#include "light.h"
#include "matrix.h"
#include "vector.h"

enum class Scene_Type {
    pbrt,
    obj,
};

struct YAR_Project {
    Scene_Type scene_type;
    std::string scene_path;

    bool has_image_resolution = false;
    Vector2i image_resolution = Vector2i{};

    bool has_render_region = false;
    Bounds2i render_region = Bounds2i{};

    bool has_camera_to_world = false;
    Matrix3x4 camera_to_world = Matrix3x4::identity;

    float world_scale = 1.f;

    // Lights defined in yar project file. The other source of lights is the
    // scene itself, for example, pbrt scene usually defines the lights. The
    // lights from yar project are merged with the native scene's lights in
    // the final Scene object.
    Lights lights; 
};

// Creates project description based on input file type.
// Supported inputs: *.yar, *.pbrt.
//
// YAR input:
//      the parsed content of yar file is used to initialize YAR_Project structure.
// PBRT input:
//      YAR_Project::scene_type = Scene_Type::pbrt, YAR_Project::scene_path = file_name, other fields take default values.
YAR_Project initialize_project(const std::string& file_name);

bool save_yar_file(const std::string& yar_file_name, const YAR_Project& project);

