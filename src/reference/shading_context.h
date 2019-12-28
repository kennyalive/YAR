#pragma once

#include "lib/light.h"
#include "lib/material.h"
#include "lib/vector.h"

struct Intersection;

// Contains all information necessary to perform shading at intersection point.
struct Shading_Context {
    Vector3 Wo; // outgoing direction
    Vector3 P; // shading point position in world coordinates
    Vector3 Ng; // geometric normal
    Vector3 N; // shading normal
    Vector2 UV; // surface UV parameterization
    Material_Handle material;
    Light_Handle area_light;

    Shading_Context(const Vector3& wo, const Intersection& intersection);
};
