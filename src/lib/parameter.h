#pragma once

#include "color.h"

template <typename Type>
struct Parameter {
    bool is_constant = false;
    Type constant_value;
    int texture_index = -1;
};

struct RGB_Parameter : Parameter<ColorRGB> {};
