#pragma once

#include "color.h"

enum class EvaluationMode {
    none,
    value,
    scale,
};

struct TextureParameter {
    int texture_index = -1;
    float u_scale = 1.f;
    float v_scale = 1.f;
};

struct LeafParameter {
    bool is_constant;
    union {
        ColorRGB constant;
        TextureParameter texture;
    };

    void set_constant(const ColorRGB& color);
    void set_constant(float value);
    void set_texture(int texture_index, float u_scale = 1.f, float v_scale = 1.f);

    LeafParameter() : is_constant(false), texture() {}

    bool operator==(const LeafParameter& other) const {
        if (is_constant) {
            return other.is_constant && constant == other.constant;
        }
        else {
            return !other.is_constant && texture.texture_index == other.texture.texture_index &&
                texture.u_scale == other.texture.u_scale && texture.v_scale == other.texture.v_scale;
        }
    }
};

struct Parameter {
    EvaluationMode eval_mode = EvaluationMode::none;

    // Leaf value. Used when eval_mode is EvaluationMode::value
    LeafParameter value;

    // References to other parameters
    int parameter0_index = -1;
    int parameter1_index = -1;
    int parameter2_index = -1;

    bool operator==(const Parameter&) const = default;
    
    void set_constant(const ColorRGB& color);
    void set_constant(float value);
    void set_texture(int texture_index);
};

struct RGB_Parameter : Parameter {};
struct Float_Parameter : Parameter {};

// TODO: deprecated?
void set_constant_parameter(Parameter& param, const ColorRGB& color);
void set_constant_parameter(Parameter& param, float value);
void set_texture_parameter(Parameter& param, int texture_index);
