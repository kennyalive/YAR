#pragma once

#include "lib/color.h"

struct Texture {
    std::vector<ColorRGB> texels;
    int width = 0;
    int height = 0;

    void init_from_file(const std::string& image_path, bool decode_srgb, bool flip_vertically);
};
