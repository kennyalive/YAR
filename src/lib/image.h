#pragma once

struct ColorRGB;

bool write_tga_image(const std::string& file_path, const std::vector<ColorRGB>& pixels, int width, int height);
