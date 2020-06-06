#pragma once

struct ColorRGB;

struct Image {
    int width = 0;
    int height = 0;
    std::vector<ColorRGB> data;

    Image() = default;
    Image(int width, int height);

    bool load_from_file(const std::string file_path, bool decode_srgb = false, bool* is_hdr_image = nullptr);
    bool write_tga(const std::string& file_path) const;
    std::vector<float> get_luminance() const;
};
