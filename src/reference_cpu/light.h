#pragma once

#include "kdtree.h"
#include "spectrum.h"
#include "lib/vector.h"

struct Local_Geometry;
struct Scene_Data;

struct Point_Light {
    Vector3  position;
    RGB     intensity;
};

struct Lights {
    std::vector<Point_Light> point_lights;
};

RGB compute_direct_lighting(
    const Local_Geometry& local_geom,
    const TwoLevel_KdTree* acceleration_structure,
    const Lights& lights,
    const Vector3& wo,
    Material_Handle material);
