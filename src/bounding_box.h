#pragma once

#include "common.h"
#include "ray.h"

struct Bounding_Box {
    Vector min_p;
    Vector max_p;

    Bounding_Box()
        : min_p(Vector(+Infinity))
        , max_p(Vector(-Infinity)) {}

    Bounding_Box(Vector min_p, Vector max_p)
        : min_p(min_p)
        , max_p(max_p) {}

    explicit Bounding_Box(Vector point)
        : min_p(point)
        , max_p(point) {}

    Bounding_Box& add_point(Vector point) {
        min_p.x = std::min(min_p.x, point.x);
        min_p.y = std::min(min_p.y, point.y);
        min_p.z = std::min(min_p.z, point.z);

        max_p.x = std::max(max_p.x, point.x);
        max_p.y = std::max(max_p.y, point.y);
        max_p.z = std::max(max_p.z, point.z);
        return *this;
    }

    bool contains(Vector point) const {
        return  point[0] >= min_p[0] && point[0] <= max_p[0] &&
                point[1] >= min_p[1] && point[1] <= max_p[1] &&
                point[2] >= min_p[2] && point[2] <= max_p[2];
    }

    bool intersect_by_ray(const Ray& ray, float& t_min, float& t_max) const {
        t_min = 0.0f;
        t_max = Infinity;

        for (int i = 0; i < 3; i++) {
            float inv_dir = 1.f / ray.d[i];
            float t0 = (min_p[i] - ray.o[i]) * inv_dir;
            float t1 = (max_p[i] - ray.o[i]) * inv_dir;

            if (inv_dir < 0.f)
                std::swap(t0, t1);

            t_min = t0 > t_min ? t0 : t_min;
            t_max = t1 < t_max ? t1 : t_max;

            if (t_min > t_max)
                return false;
        }
        return true;
    }

    static Bounding_Box get_union(const Bounding_Box& bounds, const Bounding_Box& bounds2) {
        return Bounding_Box(
            Vector(std::min(bounds.min_p.x, bounds2.min_p.x),
                   std::min(bounds.min_p.y, bounds2.min_p.y),
                   std::min(bounds.min_p.z, bounds2.min_p.z)),

            Vector(std::max(bounds.max_p.x, bounds2.max_p.x),
                   std::max(bounds.max_p.y, bounds2.max_p.y),
                   std::max(bounds.max_p.z, bounds2.max_p.z)));
    }

    static Bounding_Box get_intersection(const Bounding_Box& bounds, const Bounding_Box& bounds2) {
        Vector min_p;
        min_p.x = std::max(bounds.min_p.x, bounds2.min_p.x);
        min_p.y = std::max(bounds.min_p.y, bounds2.min_p.y);
        min_p.z = std::max(bounds.min_p.z, bounds2.min_p.z);

        Vector max_p;
        max_p.x = std::min(bounds.max_p.x, bounds2.max_p.x);
        max_p.y = std::min(bounds.max_p.y, bounds2.max_p.y);
        max_p.z = std::min(bounds.max_p.z, bounds2.max_p.z);

        return Bounding_Box(min_p, max_p);
    }
};
