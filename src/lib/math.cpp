#include "std.h"
#include "math.h"

#include "ray.h"
#include "vector.h"

template <typename T>
float solve_linear_system_2x2(float a[2][2], T b[2], T* x1, T* x2) {
    float determinant = a[0][0] * a[1][1] - a[0][1] * a[1][0];
    if (determinant != 0.f) {
        float inv_det = 1.f / determinant;
        *x1 = (a[1][1] * b[0] - a[0][1] * b[1]) * inv_det;
        *x2 = (-a[1][0] * b[0] + a[0][0] * b[1]) * inv_det;
    }
    return determinant;
}

template float solve_linear_system_2x2<float>(float a[2][2], float b[2], float* x1, float* x2);
template float solve_linear_system_2x2<Vector3>(float a[2][2], Vector3 b[2], Vector3* x1, Vector3* x2);

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

Vector3 project_vector_onto_plane_and_get_direction(const Vector3& v, const Vector3& plane_n)
{
    Vector3 t = cross(v, plane_n);
    ASSERT(t != Vector3_Zero);
    Vector3 d = cross(plane_n, t);
    return d.normalized();
}

Vector3 project_vector_onto_plane(const Vector3& v, const Vector3& plane_n)
{
    Vector3 direction = project_vector_onto_plane_and_get_direction(v, plane_n);
    Vector3 projected_v = dot(v, direction) * direction;
    return projected_v;
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

void offset_ray_origin_in_both_directions(const Vector3& p, const Vector3& geometric_normal,
    Vector3* p_adjusted_in_positive_direction, Vector3* p_adjusted_in_negative_direction)
{
    static constexpr float int_scale = 256.f;
    static constexpr float origin = 1.0 / 32.0f;
    static constexpr float float_scale = 1.0 / 65536.0f;

    const Vector3i di = Vector3i(int_scale * geometric_normal);

    int ix1 = std::bit_cast<int>(p.x) - di.x;
    int ix2 = std::bit_cast<int>(p.x) + di.x;
    p_adjusted_in_positive_direction->x = std::bit_cast<float>(p.x < 0 ? ix1 : ix2);
    p_adjusted_in_negative_direction->x = std::bit_cast<float>(p.x < 0 ? ix2 : ix1);

    int iy1 = std::bit_cast<int>(p.y) - di.y;
    int iy2 = std::bit_cast<int>(p.y) + di.y;
    p_adjusted_in_positive_direction->y = std::bit_cast<float>(p.y < 0 ? iy1 : iy2);
    p_adjusted_in_negative_direction->y = std::bit_cast<float>(p.y < 0 ? iy2 : iy1);

    int iz1 = std::bit_cast<int>(p.z) - di.z;
    int iz2 = std::bit_cast<int>(p.z) + di.z;
    p_adjusted_in_positive_direction->z = std::bit_cast<float>(p.z < 0 ? iz1 : iz2);
    p_adjusted_in_negative_direction->z = std::bit_cast<float>(p.z < 0 ? iz2 : iz1);

    if (std::abs(p.x) < origin) {
        float dx = float_scale * geometric_normal.x;
        p_adjusted_in_positive_direction->x = p.x + dx;
        p_adjusted_in_negative_direction->x = p.x - dx;
    }
    if (std::abs(p.y) < origin) {
        float dy = float_scale * geometric_normal.y;
        p_adjusted_in_positive_direction->y = p.y + dy;
        p_adjusted_in_negative_direction->y = p.y - dy;
    }
    if (std::abs(p.z) < origin) {
        float dz = float_scale * geometric_normal.z;
        p_adjusted_in_positive_direction->z = p.z + dz;
        p_adjusted_in_negative_direction->z = p.z - dz;
    }
}
