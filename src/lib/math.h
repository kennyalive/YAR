#pragma once

#include "random.h"
#include "vector.h"

struct Ray;
struct RNG;

#define ASSERT_ZERO_TO_ONE_RANGE(u) ASSERT((u) >= 0.f && (u) < 1.f)
#define ASSERT_ZERO_TO_ONE_RANGE_VECTOR2(u) ASSERT((u) >= Vector2(0.f) && (u) < Vector2(1.f))

constexpr float Pi = 3.14159265f;
constexpr float Pi2 = 6.2831853f;
constexpr float Pi_Over_2 = 1.57079632f;
constexpr float Pi_Inv = 1.f / Pi;
constexpr float Pi2_Inv = 1.f / Pi2;
constexpr float One_Minus_Epsilon = 0x1.fffffep-1;

constexpr float Infinity = std::numeric_limits<float>::infinity();

inline bool is_finite(float f) {
    return f > -Infinity && f < Infinity;
}

inline constexpr float radians(float degrees) {
    constexpr float deg_2_rad = Pi / 180.f;
    return degrees * deg_2_rad;
}

inline constexpr float degrees(float radians) {
    constexpr float rad_2_deg = 180.f / Pi;
    return radians * rad_2_deg;
}

inline bool is_power_of_2(uint32_t k) {
    return k != 0 && (k & (k - 1)) == 0;
}

inline uint32_t log2_int(uint32_t k) {
    ASSERT(k > 0);
    return most_significant_bit_index(k);
}

inline uint32_t round_up_to_power_of_2(uint32_t k) {
    ASSERT(k > 0);
    k--;
    k |= k >> 1;
    k |= k >> 2;
    k |= k >> 4;
    k |= k >> 8;
    k |= k >> 16;
    k++;
    return k;
}

template <typename T>
inline T round_up(T k, T alignment) {
    return (k + alignment - 1) & ~(alignment - 1);
}

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

inline Vector3 reflect(const Vector3& w, const Vector3& n) {
    return (2.f * dot(w, n)) * n - w;
}

inline bool refract(const Vector3& w, const Vector3& n, float etaT_over_etaI, Vector3* w_transmitted) {
    float cos_i = dot(w, n);
    float sin_i_squared = std::max(0.f, 1.f - cos_i * cos_i);

    if (sin_i_squared > etaT_over_etaI * etaT_over_etaI) 
        return false; // total internal reflection

    float etaI_over_etaT = 1.f / etaT_over_etaI;
    float cos_t = std::sqrt(std::max(0.f, 1.f - sin_i_squared * etaI_over_etaT * etaI_over_etaT));
    *w_transmitted = -etaI_over_etaT * w  + (etaI_over_etaT * cos_i - cos_t) * n;
    return true;
}

// Offsets the ray origin in the direction of the geometric normal.
// This can be used to prevent self-intersection issues when tracing a ray
// with the origin that is set to a surface point.
Vector3 offset_ray_origin(const Vector3& p, const Vector3& geometric_normal);

template <typename T>
void shuffle(T* data, int n, RNG& rng) {
    ASSERT(n > 0);
    for (int i = n; i > 1; i--) {
        uint32_t k = rng.get_bounded_uint_fast_and_biased(i);
        std::swap(data[i-1], data[k]);
    }
}
