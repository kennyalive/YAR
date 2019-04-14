#pragma once

#include "kdtree.h"
#include "light.h"
#include "lib/geometry.h"

class Camera;

struct Lights {
    std::vector<Point_Light> point_lights;
    std::vector<Diffuse_Rectangular_Light> diffuse_rectangular_lights;
};

struct Render_Context {
    Bounds2i sample_bounds;
    const Camera* camera;
    const TwoLevel_KdTree* acceleration_structure;
    Lights lights;
};
