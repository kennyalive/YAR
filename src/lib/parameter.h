#pragma once

#include "color.h"

// TODO: parameters should define more general protocol that will allow to evaluate more complex patterns.
// For example, it should be possible to evalute pbrt style materials/textures.
// Right now this implementation is an ad-hoc construct just to make some progress.

template <typename Type>
struct Parameter {
    bool is_constant = false;
    Type constant_value;
    int texture_index = -1;
};

struct RGB_Parameter : Parameter<ColorRGB> {
    float u_scale = 1.f;
    float v_scale = 1.f;
};
