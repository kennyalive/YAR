#pragma once

struct RGB;
void write_exr_image(const char* file_name, const RGB* pixels, int w, int h);
