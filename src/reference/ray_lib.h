#pragma once

#include "lib/vector.h"

// Offsets the ray origin in the direction of the geometric normal.
// This can be used to prevent self-intersection issues when tracing a ray
// with the origin that is set to a surface point.
Vector3 offset_ray_origin(const Vector3& p, const Vector3& geometric_normal);
