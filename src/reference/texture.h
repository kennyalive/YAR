#pragma once

#include "lib/color.h"

struct Vector2;

enum class Wrap_Mode {
    repeat,
    clamp
};

struct Texture {
    std::vector<ColorRGB> texels;
    int width = 0;
    int height = 0;

    void init_from_file(const std::string& image_path, bool decode_srgb, bool flip_vertically);
    ColorRGB sample_nearest(const Vector2& uv, Wrap_Mode wrap_mode) const;
    ColorRGB sample_bilinear(const Vector2& uv, Wrap_Mode wrap_mode) const;
};
