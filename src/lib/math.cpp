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
    *x1 = (b[0] * a[1][1] - a[0][1] * b[1]) * inv_det;
    *x2 = (a[0][0] * b[0] - b[1] * a[1][0]) * inv_det;
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
