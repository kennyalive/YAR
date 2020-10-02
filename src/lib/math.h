#pragma once

#include "random.h"
#include "vector.h"

struct Ray;
struct RNG;

#define ASSERT_ZERO_TO_ONE_RANGE(u) ASSERT((u) >= 0.f && (u) < 1.f)
#define ASSERT_ZERO_TO_ONE_RANGE_VECTOR2(u) ASSERT((u) >= Vector2(0.f) && (u) < Vector2(1.f))

// Solves a * x = b equation where a is 2x2 matrix, x and b are two-component vectors.
template <typename T>
bool solve_linear_system_2x2(float a[2][2], T b[2], T* x1, T* x2);

// Creates orthonormal coordinate system with axes v1, v2 and v where
// axis v is specified, v1 and v2 are chosen arbitrarily.
void coordinate_system_from_vector(const Vector3& v, Vector3* v1, Vector3* v2);

// Returns signed distance to the intersection point. Returns signed infinity if
// the ray is parallel to the plane and does not originate on the plane. If ray
// if parallel to the plane and originates on the plane the function returns 0.
float ray_plane_intersection(const Ray& ray, const Vector3& plane_n, float plane_d);

template <typename T>
inline T lerp(const T& a, const T& b, float t) {
    return (1.f - t) * a + t * b;
}

inline Vector3 get_direction_from_spherical_coordinates(float theta, float phi) {
    float sin_theta = std::sin(theta);
    return { sin_theta * std::cos(phi), sin_theta * std::sin(phi), std::cos(theta) };
}

struct Direction_Info {
    float cos_theta;
    float sin_theta;
    float cos_phi;
    float sin_phi;

    Direction_Info(const Vector3& v) {
        ASSERT(v >= Vector3(-1) && v <= Vector3(1));

        cos_theta = v.z;
        sin_theta = std::sqrt(1.f - cos_theta*cos_theta);
        ASSERT(!std::isnan(sin_theta));

        if (sin_theta == 0.f) {
            cos_phi = 1.f;
            sin_phi = 0.f;
        }
        else {
            float sin_theta_inv = 1.f / sin_theta;
            cos_phi = std::clamp(v.x * sin_theta_inv, -1.f, 1.f);
            sin_phi = std::clamp(v.y * sin_theta_inv, -1.f, 1.f);
        }
    }
};

inline Vector3 reflect(const Vector3& v, const Vector3& n) {
    return (2.f * dot(v, n)) * n - v;
}

// Offsets the ray origin in the direction of the geometric normal.
// This can be used to prevent self-intersection issues when tracing a ray
// with the origin that is set to a surface point.
Vector3 offset_ray_origin(const Vector3& p, const Vector3& geometric_normal);

template <typename T>
void shuffle(T* first, T* last, RNG& rng) {
    uint32_t n = uint32_t(last - first);
    ASSERT(n > 0);
    for (uint32_t i = 0; i < n-1; i++) {
        uint32_t k = i + rng.get_bounded_uint(n - i);
        std::swap(first[i], first[k]);
    }
}
