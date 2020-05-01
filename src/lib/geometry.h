#pragma once

#include "triangle_mesh.h"

enum class Geometry_Type : uint32_t {
    none,
    triangle_mesh,
    sphere
};

struct Geometry_Handle {
    Geometry_Type type = Geometry_Type::none;
    int index = -1;

    bool operator==(const Geometry_Handle& other) const {
        return type == other.type && index == other.index;
    }
    bool operator!=(const Geometry_Handle& other) const {
        return !(*this == other);
    }
};

struct Sphere {
    float radius = 0.f;
    Vector3 origin;
};

struct Geometries {
    std::vector<Triangle_Mesh> triangle_meshes;
    std::vector<Sphere> spheres;
};

static_assert(sizeof(Geometry_Handle) == 8);
constexpr Geometry_Handle Null_Geometry = {Geometry_Type::none, -1};
