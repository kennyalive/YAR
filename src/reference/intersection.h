#pragma once

#include "light.h"
#include "lib/material.h"
#include "lib/vector.h"

struct Ray;
struct Triangle_Mesh;

float intersect_triangle_moller_trumbore(const Ray& ray, const Vector3& p0, const Vector3& p1, const Vector3& p2, float& b1, float& b2);

struct Triangle_Intersection {
    float                   t = Infinity; // If no intersection is found only this field is initialized and it is set to Infinity.
    float                   b1;
    float                   b2;
    const Triangle_Mesh*    mesh;
    int32_t                 triangle_index;
};

void intersect_triangle(const Ray& ray, const Triangle_Mesh* mesh, int32_t triangle_index, Triangle_Intersection& intersection);

struct Local_Geometry {
    Vector3                 position;
    Vector3                 normal;
    Material_Handle         material; 
    Light_Handle            area_light;

    Local_Geometry() {}
    explicit Local_Geometry(const Ray& ray, const Triangle_Intersection& triangle_intersection);
};
