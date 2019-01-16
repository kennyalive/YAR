#pragma once

#include "lib/vector.h"

// Uniformly samples uniform sphere.
// u0, u1 - uniformly distributed random variables.
Vector3 uniform_sample_sphere(float u0, float u1);