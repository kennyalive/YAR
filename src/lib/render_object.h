#pragma once

#include "triangle_mesh.h"
#include "reference/light.h"
#include "lib/material.h"

enum class Geometry_Type : uint32_t {
    none,
    triangle_mesh
};

struct Geometry_Handle {
    Geometry_Type type;
    int index;
};

struct Geometries {
    std::vector<Triangle_Mesh> triangle_meshes;
};

static_assert(sizeof(Geometry_Handle) == 8);
constexpr Geometry_Handle Null_Geometry = { Geometry_Type::none, -1 };

struct Render_Object {
    Geometry_Handle geometry = Null_Geometry;
    Material_Handle material = Null_Material;
    Light_Handle area_light = Null_Light;
    Matrix3x4 world_to_object_transform;
    Matrix3x4 object_to_world_transform;
};

