#include "common.h"
#include "vector.h"

#include <cmath>

Vector uniform_sample_sphere(float u0, float u1)
{
    float z = 1.f - 2.f * u0;
    float r = std::sqrt(1.f - z*z);
    float phi = 2.f * Pi * u1;
    return Vector(r * std::cos(phi), r * std::sin(phi), z);
}