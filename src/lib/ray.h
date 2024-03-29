#pragma once

#include "vector.h"

struct Ray {
    Vector3 origin;
    Vector3 direction;

    Vector3 get_point(float t) const {
        return origin + direction * t;
    }
    bool operator==(const Ray& other) const {
        return origin == other.origin && direction == other.direction;
    }
    bool operator!=(const Ray& other) const {
        return !(*this == other);
    }
};

// Additional rays around the main ray that allow to determine the size of the pixel footprint on the surface.
// It is used to compute UV derivatives with respect to pixel coordinates and ultimately texture lod level.
struct Differential_Rays {
    Ray dx_ray;
    Ray dy_ray;
};
