#pragma once

#include "color.h"

enum class EvaluationMode {
    none,
    constant,
    texture,
};

template <typename Type>
struct Parameter {
    EvaluationMode eval_mode = EvaluationMode::none;

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
    param.eval_mode = EvaluationMode::constant;
    param.constant_value = value;
}

template <typename Parameter_Type>
void set_texture_parameter(Parameter_Type& param, int texture_index) {
    param.eval_mode = EvaluationMode::texture;
    param.texture_index = texture_index;
}
