#pragma once

#include "color.h"

// TODO: parameters should define more general protocol that will allow to evaluate more complex patterns.
// For example, it should be possible to evalute pbrt style materials/textures.
// Right now this implementation is an ad-hoc construct just to make some progress.

template <typename Type>
struct Parameter {
    bool is_specified = false;
    bool is_constant = false;
    Type constant_value = Type();
    int texture_index = -1;

    float u_scale = 1.f;
    float v_scale = 1.f;

    bool operator==(const Parameter&) const = default;
};

struct RGB_Parameter : Parameter<ColorRGB> {
    bool operator==(const RGB_Parameter&) const = default;
};

struct Float_Parameter : Parameter<float> {

};

template <typename Parameter_Type, typename Type>
void set_constant_parameter(Parameter_Type& param, const Type& value) {
    param.is_specified = true;
    param.is_constant = true;
    param.constant_value = value;
}

template <typename Parameter_Type>
void set_texture_parameter(Parameter_Type& param, int texture_index) {
    param.is_specified = true;
    param.is_constant = false;
    param.texture_index = texture_index;
}
