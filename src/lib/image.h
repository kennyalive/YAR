#pragma once

struct ColorRGB;

struct Image {
    int width = 0;
    int height = 0;
    std::vector<ColorRGB> data;

    Image() = default;
    Image(int width, int height);
    bool write_tga(const std::string& file_path) const;
};
