#include "std.h"
#include "lib/common.h"
#include "ray_lib.h"

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
