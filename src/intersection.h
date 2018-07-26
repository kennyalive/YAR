#pragma once

#include "common.h"
#include "vector.h"

struct Ray;

float intersect_triangle_moller_trumbore(const Ray& ray, const Vector& p0, const Vector& p1, const Vector& p2, float& b1, float& b2);

struct Triangle_Intersection {
    float                   t = Infinity; // If no intersection is found only this field is initialized and it is set to Infinity.
    float                   b1;
    float                   b2;
    const Triangle_Mesh*    mesh;
    int32_t                 triangle_index;
};

Triangle_Intersection intersect_triangle(const Ray& ray, const Triangle_Mesh* mesh, int32_t triangle_index);

struct Local_Geometry {
    Vector                  position;
    Vector                  normal;

    Local_Geometry() {}
    explicit Local_Geometry(const Ray& ray, const Triangle_Intersection& triangle_intersection);
};
