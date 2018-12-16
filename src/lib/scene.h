#pragma once

#include "light.h"
#include "mesh.h"
#include <vector>

struct Scene_Data {
    std::vector<Mesh_Data> meshes;
    std::vector<Point_Light_Data> point_lights;
};
