#pragma once

#include "bounding_box.h"
#include "matrix.h"
#include "scene.h"
#include "vector.h"

enum class Project_Type {
    test,
    pbrt
};

struct YAR_Project {
    Project_Type type;
    std::string path;
    Vector2i image_resolution;
    Bounds2i render_region;
    Matrix3x4 camera_to_world;
};

YAR_Project parse_project(const std::string& file_name);
bool save_project(const std::string& file_name, const YAR_Project& project);
Scene load_project(const YAR_Project& project);

