#pragma once

#include "kdtree.h"

#include "lib/color.h"
#include "lib/vector.h"

struct Local_Geometry;
struct Scene_Data;

struct Point_Light {
    Vector3     position;
    ColorRGB    intensity;
};

struct Lights {
    std::vector<Point_Light> point_lights;
};

ColorRGB compute_direct_lighting(
    const Local_Geometry& local_geom,
    const TwoLevel_KdTree* acceleration_structure,
    const Lights& lights,
    const Vector3& wo,
    Material_Handle material);
