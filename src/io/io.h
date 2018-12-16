#pragma once

struct Vector3;

void write_exr_image(const char* file_name, const Vector3* pixels, int w, int h);
