#pragma once

#include "geometry.h"
#include "light.h"
#include "material.h"
#include "matrix.h"

struct Scene_Object {
    Geometry_Handle geometry;
    Material_Handle material;
    Light_Handle area_light;
    Matrix3x4 object_to_world_transform;
    Matrix3x4 world_to_object_transform;
};
