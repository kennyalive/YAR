#include "std.h"
#include "lib/common.h"
#include "material_parameter.h"

void LeafParameter::set_constant(const ColorRGB& color) {
    is_constant = true;
    constant = color;
}

void LeafParameter::set_constant(float value) {
    is_constant = true;
    constant = ColorRGB(value);
}

void LeafParameter::set_texture(int texture_index, float u_scale, float v_scale) {
    is_constant = false;
    texture.texture_index = texture_index;
    texture.u_scale = u_scale;
    texture.v_scale = v_scale;
}

void Parameter::set_constant(const ColorRGB& color) {
    eval_mode = EvaluationMode::value;
    value.set_constant(color);
}

void Parameter::set_constant(float value) {
    eval_mode = EvaluationMode::value;
    this->value.set_constant(value);
}

void Parameter::set_texture(int texture_index) {
    eval_mode = EvaluationMode::value;
    value.set_texture(texture_index);
}

void set_constant_parameter(Parameter& param, const ColorRGB& color) {
    param.set_constant(color);
}

void set_constant_parameter(Parameter& param, float value) {
    param.set_constant(value);
}

void set_texture_parameter(Parameter& param, int texture_index) {
    param.set_texture(texture_index);
}
