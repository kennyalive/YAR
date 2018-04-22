#pragma once

#include "vector.h"

struct Ray {
    Vector o; // origin
    Vector d; // direction

    Ray(Vector origin, Vector direction)
        : o(origin), d(direction) {}
};
