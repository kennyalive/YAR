#pragma once

#include "triangle_mesh.h"

enum class Geometry_Type : uint32_t {
    triangle_mesh,
    count,
    null_geometry = std::numeric_limits<uint32_t>::max()
};

struct Geometry_Handle {
    Geometry_Type type = Geometry_Type::null_geometry;
    int index = -1;

    bool operator==(const Geometry_Handle& other) const {
        return type == other.type && index == other.index;
    }
    bool operator!=(const Geometry_Handle& other) const {
        return !(*this == other);
    }
};

struct Geometries {
    std::vector<Triangle_Mesh> triangle_meshes;
};

static_assert(sizeof(Geometry_Handle) == 8);
constexpr Geometry_Handle Null_Geometry = {Geometry_Type::null_geometry, -1};
constexpr int Geometry_Type_Count = static_cast<int>(Geometry_Type::count);
