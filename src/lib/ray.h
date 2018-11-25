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
};
