#pragma once

#include "lib/color.h"

struct Texture {
    void init_from_file(const std::string& image_path);
    std::vector<ColorRGB> texels;
};
