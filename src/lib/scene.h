#pragma once

#include "geometry.h"
#include "light.h"
#include "material.h"
#include "matrix.h"
#include "render_object.h"

struct YAR_Project;

struct Scene {
    std::string project_dir;
    Geometries geometries;
    Materials materials;
    Lights lights;
    std::vector<Render_Object> render_objects;
    std::vector<Matrix3x4> view_points; // predefined camera positions
};

Scene create_scene(const YAR_Project& project);

