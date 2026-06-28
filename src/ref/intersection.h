#pragma once

#include "lib/geometry.h"

struct Ray;
struct Scene_Object;

float intersect_triangle_möller_trumbore(const Ray& ray, const Vector3& p0, const Vector3& p1, const Vector3& p2, Vector3* barycentrics);
float intersect_triangle_watertight(const Ray& ray, const Vector3& p0, const Vector3& p1, const Vector3& p2, Vector3* barycentrics);

struct Triangle_Intersection {
    const Triangle_Mesh* mesh = nullptr;
    Vector3 barycentrics;
    uint32_t triangle_index = (uint32_t)-1;
};

struct Intersection {
    // Distance to the intersection point. Initial value defines ray's [0, t_max) range to check for intersections.
    float t = Infinity;

    // Type of the intersected geometry.
    Geometry_Type geometry_type = Geometry_Type::null_geometry;

    // Scene_Object associated with the intersected geometry.
    // It is null for Geometry_KdTree and it is non-null for Scene_KdTree.
    const Scene_Object* scene_object = nullptr;

    Triangle_Intersection triangle_intersection;
};
