#pragma once

#include "geometry.h"
#include "light.h"
#include "material.h"
#include "matrix.h"
#include "scene_object.h"

enum class Scene_Type {
    pbrt,
    obj,
};

struct Scene {
    Scene_Type type;
    std::string path;

    //
    // Renderer configuration
    //
    Vector2i image_resolution;
    Bounds2i render_region;
    float camera_fov_y = 0.f;
    bool  z_is_up = false;
    bool mesh_disable_backfacing_culling = false;
    bool front_face_has_clockwise_winding = false;
    
    // Predefined camera positions.
    std::vector<Matrix3x4> view_points;

    //
    // Description of the virtual environment.
    //
    Geometries geometries;
    Materials materials;
    Lights lights;
    std::vector<Scene_Object> objects;
};
