#pragma once

#include "triangle_mesh.h"
#include "geometry.h"
#include "light.h"
#include "material.h"
#include "matrix.h"

struct Render_Object {
    Geometry_Handle geometry = Null_Geometry;
    Material_Handle material = Null_Material;
    Light_Handle area_light = Null_Light;
    Matrix3x4 object_to_world_transform;
    Matrix3x4 world_to_object_transform;
};

