#pragma once

#include "vector.h"

struct Ray {
    Vector3 origin;
    Vector3 direction;

    Ray() {}
    Ray(Vector3 origin, Vector3 direction)
        : origin(origin), direction(direction) {}

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
struct Auxilary_Rays {
    Ray ray_dx_offset;
    Ray ray_dy_offset;
};
