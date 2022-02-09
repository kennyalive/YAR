#pragma once

struct ColorRGB;
struct _EXRAttribute; // from tinyexr library

struct Image {
    int width = 0;
    int height = 0;
    std::vector<ColorRGB> data;

    Image() = default;
    Image(int width, int height);

    bool load_from_file(const std::string file_path, bool decode_srgb = false, bool* is_hdr_image = nullptr);
    bool write_tga(const std::string& file_path) const;
    bool write_exr(const std::string& file_path, const std::vector<_EXRAttribute>& custom_attributes) const;

    void flip_horizontally();
};
