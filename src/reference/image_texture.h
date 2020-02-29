#pragma once

#include "lib/color.h"

struct Vector2;

enum class Wrap_Mode {
    repeat,
    clamp
};

class Image_Texture {
public:
    struct Init_Params {
        bool generate_mips = true;
        bool decode_srgb = true;
        bool flip_vertically = false;
    };

    void initialize_from_file(const std::string& image_path, const Init_Params& params);
    ColorRGB sample_nearest(const Vector2& uv, Wrap_Mode wrap_mode) const;
    ColorRGB sample_bilinear(const Vector2& uv, Wrap_Mode wrap_mode) const;

private:
    void upsample_base_level_to_power_of_two_resolution();
    void generate_mips();

private:
    int width = 0;
    int height = 0;
    std::vector<std::vector<ColorRGB>> mips;
};
