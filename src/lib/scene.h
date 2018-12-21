#pragma once

#include "light.h"
#include "mesh.h"
#include <vector>

struct Scene_Data {
    std::vector<Mesh_Data> meshes;
    std::vector<RGB_Point_Light_Data> rgb_point_lights;
};
