#pragma once

#include "kdtree.h"

#include "io/io.h"

#include "lib/color.h"
#include "lib/matrix.h"
#include "lib/random.h"
#include "lib/vector.h"

struct Local_Geometry;

struct Point_Light {
    Vector3 position;
    ColorRGB intensity;
};

struct Diffuse_Rectangular_Light {
    Matrix3x4 light_to_world_transform;
    ColorRGB emitted_radiance;
    Vector2 size;
    float area;
    int shadow_ray_count;

    Diffuse_Rectangular_Light(const RGB_Diffuse_Rectangular_Light_Data& light_data);
};

struct Lights {
    std::vector<Point_Light> point_lights;
    std::vector<Diffuse_Rectangular_Light> diffuse_rectangular_lights;
};

ColorRGB compute_direct_lighting(
    const Local_Geometry& local_geom,
    const TwoLevel_KdTree* acceleration_structure,
    const Lights& lights,
    const Vector3& wo,
    Material_Handle material,
    pcg32_random_t* rng);
