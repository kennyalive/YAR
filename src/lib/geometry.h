#pragma once

#include "vector.h"

struct Bounds2i {
    Vector2i p0; // inclusive
    Vector2i p1; // exclusive

    Vector2i size() const {
        return p1 - p0;
    }

    int area() const {
        Vector2i d = size();
        return d.x * d.y;
    }
};

inline Bounds2i intersect_bounds(const Bounds2i& a, const Bounds2i& b) {
    return Bounds2i {
        Vector2i{ std::max(a.p0.x, b.p0.x), std::max(a.p0.y, b.p0.y) },
        Vector2i{ std::min(a.p1.x, b.p1.x), std::min(a.p1.y, b.p1.y) }
    };
}

inline bool is_inside_bounds(const Bounds2i& b, Vector2i p) {
    return p >= b.p0 && p < b.p1;
}
