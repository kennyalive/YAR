#pragma once

#include "lib/color.h"
#include "lib/image.h"

struct Vector2;

enum class Wrap_Mode {
    repeat,
    clamp
};

enum class Filter_Type {
    box,
    lanczos2,
    lanczos3,
    kaiser2_alpha_4,
    kaiser3_alpha_4,
    mitchell_B_1_3_C_1_3,
};

class Image_Texture {
public:
    struct Init_Params {
        bool generate_mips = false;
        Filter_Type mip_filter = Filter_Type::lanczos2;
        bool decode_srgb = false;
        float scale = 1.f;
    };

    void initialize_from_file(const std::string& image_path, const Init_Params& params);
    void initialize_from_constant_value(const ColorRGB& color);

    const std::vector<Image>& get_mips() const { return mips; }

    ColorRGB sample_nearest(const Vector2& uv, int mip_level, Wrap_Mode wrap_mode) const;
    ColorRGB sample_bilinear(const Vector2& uv, int mip_level, Wrap_Mode wrap_mode) const;
    ColorRGB sample_trilinear(const Vector2& uv, float lod, Wrap_Mode wrap_mode) const;
    ColorRGB sample_EWA(Vector2 uv, Vector2 uv_axis1, Vector2 uv_axis2, Wrap_Mode wrap_mode, float max_anisotropy) const;

private:
    void upsample_base_level_to_power_of_two_resolution(bool is_hdr_image);
    void generate_mips(Filter_Type filter, bool is_hdr_image);

private:
    std::vector<Image> mips;
};
