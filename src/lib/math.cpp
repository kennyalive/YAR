#include "std.h"
#include "math.h"

#include "ray.h"
#include "vector.h"

template <typename T>
bool solve_linear_system_2x2(float a[2][2], T b[2], T* x1, T* x2) {
    float det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (det == 0.f)
        return false;
    float inv_det = 1.f / det;
    *x1 = ( a[1][1] * b[0] - a[0][1] * b[1]) * inv_det;
    *x2 = (-a[1][0] * b[0] + a[0][0] * b[1]) * inv_det;
    return true;
}

template bool solve_linear_system_2x2<float>(float a[2][2], float b[2], float* x1, float* x2);
template bool solve_linear_system_2x2<Vector3>(float a[2][2], Vector3 b[2], Vector3* x1, Vector3* x2);

void coordinate_system_from_vector(const Vector3& v, Vector3* v1, Vector3* v2) {
    *v1 = (std::abs(v.x) > std::abs(v.y)) ? Vector3(-v.z, 0, v.x) : Vector3(0, -v.z, v.y);
    v1->normalize();
    *v2 = cross(v, *v1);
}

float ray_plane_intersection(const Ray& ray, const Vector3& plane_n, float plane_d) {
    float k = -dot(ray.origin, plane_n) - plane_d;
    if (k == 0) return 0.f; // to prevent potential NaN during division
    return k / dot(ray.direction, plane_n);
}

static int float_bits_to_int(float f) {
    static_assert(sizeof(float) == sizeof(int));
    int i;
    memcpy(&i, &f, sizeof(f));
    return i;
}

static float int_bits_to_float(int i) {
    static_assert(sizeof(float) == sizeof(int));
    float f;
    memcpy(&f, &i, sizeof(i));
    return f;
}

// Ray Tracing Gems, chapter 6: A Fast and Robust Method for Avoiding Self-Intersection.
Vector3 offset_ray_origin(const Vector3& p, const Vector3& geometric_normal) {
    const float int_scale = 256.f;
    Vector3i of_i = Vector3i(int_scale * geometric_normal);

    Vector3 p_i;
    p_i.x = int_bits_to_float(float_bits_to_int(p.x) + ((p.x < 0) ? -of_i.x : of_i.x));
    p_i.y = int_bits_to_float(float_bits_to_int(p.y) + ((p.y < 0) ? -of_i.y : of_i.y));
    p_i.z = int_bits_to_float(float_bits_to_int(p.z) + ((p.z < 0) ? -of_i.z : of_i.z));

    const float origin = 1.0/32.0f;
    const float float_scale = 1.0/65536.0f;
    Vector3 p_adjusted;
    p_adjusted.x = std::abs(p.x) < origin ? (p.x + float_scale*geometric_normal.x) : p_i.x;
    p_adjusted.y = std::abs(p.y) < origin ? (p.y + float_scale*geometric_normal.y) : p_i.y;
    p_adjusted.z = std::abs(p.z) < origin ? (p.z + float_scale*geometric_normal.z) : p_i.z;
    return p_adjusted;
}
