#pragma once

#include "geometry.h"
#include "light.h"
#include "material.h"
#include "matrix.h"
#include "render_object.h"

struct YAR_Project;

struct Scene {
    Geometries geometries;
    Materials materials;
    Lights lights;
    std::vector<Render_Object> render_objects;
    // Predefined camera positions.
    std::vector<Matrix3x4> view_points; 
    float fovy{ 0.f };
    bool front_face_has_clockwise_winding{ false };
};

Scene load_scene(const YAR_Project& project);

