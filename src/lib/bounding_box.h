#pragma once

#include "math.h"
#include "ray.h"

struct Bounding_Box {
    Vector3 min_p;
    Vector3 max_p;

    Bounding_Box()
        : min_p(Vector3(+Infinity))
        , max_p(Vector3(-Infinity)) {}

    Bounding_Box(Vector3 min_p, Vector3 max_p)
        : min_p(min_p)
        , max_p(max_p) {}

    explicit Bounding_Box(Vector3 point)
        : min_p(point)
        , max_p(point) {}

    Bounding_Box& add_point(Vector3 point) {
        min_p.x = std::min(min_p.x, point.x);
        min_p.y = std::min(min_p.y, point.y);
        min_p.z = std::min(min_p.z, point.z);

        max_p.x = std::max(max_p.x, point.x);
        max_p.y = std::max(max_p.y, point.y);
        max_p.z = std::max(max_p.z, point.z);
        return *this;
    }

    bool contains(Vector3 point) const {
        return  point[0] >= min_p[0] && point[0] <= max_p[0] &&
                point[1] >= min_p[1] && point[1] <= max_p[1] &&
                point[2] >= min_p[2] && point[2] <= max_p[2];
    }

    bool intersect_by_ray(const Ray& ray, float* t_min, float* t_max) const {
        float t0 = 0.f, t1 = Infinity; // [t0, t1] tracks current ray segment
        for (int i = 0; i < 3; i++) {
            float inv_dir = 1.f / ray.direction[i];
            float slab_t0 = (min_p[i] - ray.origin[i]) * inv_dir;
            float slab_t1 = (max_p[i] - ray.origin[i]) * inv_dir;
            if (inv_dir < 0.f)
                std::swap(slab_t0, slab_t1);

            // Intersect ranges [t0, t1] and [slab_t0, slab_t1]
            t0 = slab_t0 > t0 ? slab_t0 : t0;
            t1 = slab_t1 < t1 ? slab_t1 : t1;

            if (t0 > t1) // check for empty segment, which means there is no intersection
                return false;
        }
        *t_min = t0;
        *t_max = t1;
        return true;
    }

    // Equivalent to intersect_by_ray() but does not produce NaNs during intermediate computations.
    // This could be useful when ENABLE_INVALID_FP_EXCEPTION is enabled, which might trigger hardware
    // floating point exception in intersect_by_ray() even though NaNs are handled properly there.
    bool intersect_by_ray_without_NaNs(const Ray& ray, float* t_min, float* t_max) const {
        float t0 = 0.f, t1 = Infinity;
        for (int i = 0; i < 3; i++) {
            if (ray.direction[i] != 0.f) {
                float inv_dir = 1.f / ray.direction[i];
                float slab_t0 = (min_p[i] - ray.origin[i]) * inv_dir;
                float slab_t1 = (max_p[i] - ray.origin[i]) * inv_dir;
                if (inv_dir < 0.f)
                    std::swap(slab_t0, slab_t1);

                t0 = slab_t0 > t0 ? slab_t0 : t0;
                t1 = slab_t1 < t1 ? slab_t1 : t1;

                if (t0 > t1)
                    return false;
            }
            else if (ray.origin[i] < min_p[i] || ray.origin[i] > max_p[i])
                return false;
        }
        *t_min = t0;
        *t_max = t1;
        return true;
    }

    static Bounding_Box compute_union(const Bounding_Box& bounds, const Bounding_Box& bounds2) {
        return Bounding_Box(
            Vector3(std::min(bounds.min_p.x, bounds2.min_p.x),
                   std::min(bounds.min_p.y, bounds2.min_p.y),
                   std::min(bounds.min_p.z, bounds2.min_p.z)),

            Vector3(std::max(bounds.max_p.x, bounds2.max_p.x),
                   std::max(bounds.max_p.y, bounds2.max_p.y),
                   std::max(bounds.max_p.z, bounds2.max_p.z)));
    }

    static Bounding_Box compute_intersection(const Bounding_Box& bounds, const Bounding_Box& bounds2) {
        Vector3 min_p;
        min_p.x = std::max(bounds.min_p.x, bounds2.min_p.x);
        min_p.y = std::max(bounds.min_p.y, bounds2.min_p.y);
        min_p.z = std::max(bounds.min_p.z, bounds2.min_p.z);

        Vector3 max_p;
        max_p.x = std::min(bounds.max_p.x, bounds2.max_p.x);
        max_p.y = std::min(bounds.max_p.y, bounds2.max_p.y);
        max_p.z = std::min(bounds.max_p.z, bounds2.max_p.z);

        return Bounding_Box(min_p, max_p);
    }
};

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

    bool operator==(const Bounds2i& other) const {
        return p0 == other.p0 && p1 == other.p1;
    }
    bool operator!=(const Bounds2i& other) const {
        return !(*this == other);
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

