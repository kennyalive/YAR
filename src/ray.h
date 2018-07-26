#pragma once

#include "vector.h"

struct Ray {
    Vector origin;
    Vector direction;

    Ray() {}
    Ray(Vector origin, Vector direction)
        : origin(origin), direction(direction) {}

    Vector get_point(float t) const {
        return origin + direction * t;
    }
};
