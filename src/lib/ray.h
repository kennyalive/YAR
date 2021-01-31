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

struct Footprint_Tracking_Ray {
    Ray main_ray;
    Ray auxilary_ray_dx_offset;
    Ray auxilary_ray_dy_offset;
};
