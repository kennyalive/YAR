#pragma once

#include "triangle_mesh.h"
#include "reference/light.h"
#include "lib/material.h"

enum class Geometry_Type : uint32_t {
    node,
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
constexpr Geometry_Handle Null_Geometry = { Geometry_Type::node, -1 };

struct Render_Object {
    Geometry_Handle geometry = Null_Geometry;
    Material_Handle material = Null_Material;
    Light_Handle area_light = Null_Light;
};
