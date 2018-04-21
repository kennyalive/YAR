#pragma once

#include "triangle.h"

#include <array>
#include <limits>

struct Ray;
struct Vector;

using Triangle = std::array<Vector, 3>;

struct Triangle_Intersection {
    float t = std::numeric_limits<float>::infinity();
    float b1 = 0.0;
    float b2 = 0.0;
};

bool intersect_triangle(const Ray& ray, const Triangle& triangle, Triangle_Intersection& intersection);
