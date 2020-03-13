#pragma once

#include "lib/color.h"

struct Vector2;

enum class Wrap_Mode {
    repeat,
    clamp
};

enum class Filter_Type {
    lanczos2,
    lanczos3,
    kaiser2_alpha_4,
    kaiser3_alpha_4,
    mitchell_B_1_3_C_1_3
};

class Image_Texture {
public:
    struct Init_Params {
        bool generate_mips = true;
        Filter_Type mip_filter = Filter_Type::lanczos2;
        bool decode_srgb = true;
        bool flip_vertically = false;
    };

    void initialize_from_file(const std::string& image_path, const Init_Params& params);
    ColorRGB sample_nearest(const Vector2& uv, Wrap_Mode wrap_mode) const;
    ColorRGB sample_bilinear(const Vector2& uv, Wrap_Mode wrap_mode) const;

private:
    void upsample_base_level_to_power_of_two_resolution();
    void generate_mips(Filter_Type filter);

private:
    int width = 0;
    int height = 0;
    std::vector<std::vector<ColorRGB>> mips;
};
