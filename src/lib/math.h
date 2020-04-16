#pragma once

struct Ray;
struct Vector3;

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
