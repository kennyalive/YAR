#pragma once

#include "lib/math.h"

struct Vector3;

struct Triangle_Intersection_8x {
    __m256 t;  // distance to intersection point or Infinity if no intersection
    __m256 bx; // barycentric coordinates x/y/z
    __m256 by;
    __m256 bz;
    __m256i triangle_index;

    static Triangle_Intersection_8x no_intersection()
    {
        Triangle_Intersection_8x result;
        result.t = _mm256_set1_ps(Infinity);
        return result;
    }

    void min(const Triangle_Intersection_8x& other);
    void reduce(float* distance, Vector3* barycentrics, uint32_t* primitive_index) const;
};

Triangle_Intersection_8x intersect_triangle_watertight_8x(const Ray& ray, const __m256 px[3], const __m256 py[3], const __m256 pz[3]);

struct Triangle_Intersection_4x {
    __m128 t;  // distance to intersection point or Infinity if no intersection
    __m128 bx; // barycentric coordinates x/y/z
    __m128 by;
    __m128 bz;
    __m128i triangle_index;

    static Triangle_Intersection_4x no_intersection()
    {
        Triangle_Intersection_4x result;
        result.t = _mm_set1_ps(Infinity);
        return result;
    }

    void min(const Triangle_Intersection_4x& other);
    void reduce(float* distance, Vector3* barycentrics, uint32_t* primitive_index) const;
};

Triangle_Intersection_4x intersect_triangle_watertight_4x(const Ray& ray, const __m128 px[3], const __m128 py[3], const __m128 pz[3]);
