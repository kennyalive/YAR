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

enum class Raytracer_Renderer_Type {
    direct_lighting,
    path_tracer
};

struct Raytracer_Renderer_Config {
    Raytracer_Renderer_Type type = Raytracer_Renderer_Type::path_tracer; 

    // The path length is defined as the number of line segments that connect camera and light source for
    // specific light path. This limit is used by the algorithms that try to compute rendering equation
    // integral (for example, path tracing). It is also used in direct lighting rendering when we have
    // longer light passes due to specular reflection and refraction.
    //
    // 0 - light is not emitted, complete darkness
    // 1 - only emmited light
    // 2 - direct lighting
    // 3 - first bounce of indirect lighting
    // 4 - second bound of indirect lighting
    // ...
    int max_path_length = 100;
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
    int x_pixel_samples = 0;
    int y_pixel_samples = 0;
    Raytracer_Renderer_Config raytracer_config;

    // Predefined camera positions.
    std::vector<Matrix3x4> view_points;

    // The list of textures used in the scene.
    std::vector<std::string> texture_names;

    //
    // Description of the virtual environment.
    //
    Geometries geometries;
    Materials materials;
    Lights lights;
    std::vector<Scene_Object> objects;
};
