#pragma once

#include "lib/color.h"

struct Vector3;

class Bsdf {
    virtual ColorRGB evaluate(const Vector3& wo, const Vector3& wi) const = 0;
};
