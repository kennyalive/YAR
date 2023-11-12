#pragma once

#include "color.h"

struct _EXRAttribute; // from tinyexr library

struct Image {
    int width = 0;
    int height = 0;
    std::vector<ColorRGB> data;

    Image() = default;
    Image(int width, int height);

    bool load_from_file(const std::string file_path, bool decode_srgb = false, bool* is_hdr_image = nullptr);
    void init_from_constant_value(int width, int height, const ColorRGB color);
    bool write_tga(const std::string& file_path) const;
    bool write_exr(const std::string& file_path, bool compress_image, const std::vector<_EXRAttribute>& custom_attributes) const;

    void extend_to_region(Vector2i size, Vector2i offset);
    void flip_horizontally();
};
