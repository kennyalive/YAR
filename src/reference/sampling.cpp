#include "lib/common.h"
#include "lib/vector.h"

#include <cmath>

Vector3 uniform_sample_sphere(float u0, float u1)
{
    float z = 1.f - 2.f * u0;
    float r = std::sqrt(1.f - z*z);
    float phi = 2.f * Pi * u1;
    return Vector3(r * std::cos(phi), r * std::sin(phi), z);
}
