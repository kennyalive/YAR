#pragma once

#include "lib/geometry.h"

struct Intersection;
struct Ray;
struct Render_Object;

float intersect_triangle_moller_trumbore(const Ray& ray, const Vector3& p0, const Vector3& p1, const Vector3& p2, float& b1, float& b2);

// Intersection test between a ray and a geometric primitive (i.e. a triangle).
// The ray's parametric range is restricted to the half-open interval [0, t_max),
// where t_max is defined by the initial value of Intersection::t.
// If intersection is found then Intersection::t gets overwritten with a distance
// to the intersection point, otherwise Intersection::t is unchanged.
void intersect_geometric_primitive(
    const Ray& ray,
    const Geometries* geometries, Geometry_Handle geometry, int primitive_index,
    Intersection& intersection);

struct Triangle_Intersection {
    float b1;
    float b2;
    const Triangle_Mesh* mesh;
    int triangle_index;
};

struct Intersection {
    // Distance to the intersection point. Initial value defines ray's [0, t_max) range to check for intersections.
    float t = Infinity;

    // Type of the intersected geometry.
    Geometry_Type geometry_type = Geometry_Type::none;

    // Render_Object associated with the intersected geometry. It is always null for Geometry_KdTree
    // and it is always non-null for Scene_KdTree.
    const Render_Object* render_object = nullptr;

    union {
        Triangle_Intersection triangle_intersection;
    };
};
