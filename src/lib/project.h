#pragma once

#include "bounding_box.h"
#include "matrix.h"
#include "vector.h"

enum class Scene_Type {
    pbrt,
    test
};

struct YAR_Project {
    Scene_Type scene_type;
    std::string scene_path;

    Vector2i image_resolution;
    Bounds2i render_region;
    Matrix3x4 camera_to_world;
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

