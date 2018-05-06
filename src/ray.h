#pragma once

#include "vector.h"

struct Ray {
    Vector o; // origin
    Vector d; // direction

    Ray() {}
    Ray(Vector origin, Vector direction)
        : o(origin), d(direction) {}

    Vector get_point(float t) const {
        return o + d * t;
    }
};
