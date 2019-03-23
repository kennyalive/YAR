#include "std.h"
#include "common.h"
#include "io.h"

#include "half/half.h"

YAR_File load_yar_file(const std::string& path) {
    ASSERT(false);
    YAR_File result;
    return result;
}

Scene_Data load_scene(Scene_Type scene_type, const std::string& scene_path) {
    ASSERT(false);
    Scene_Data scene_data;
    return scene_data;
}

// from third_party/miniexr.cpp
unsigned char* miniexr_write(unsigned width, unsigned height, unsigned channels, const void* rgba16f, size_t* out_size);

void write_exr_image(const char* file_name, const ColorRGB* pixels, int w, int h) {
    FILE* file;
    if (fopen_s(&file, file_name, "wb") != 0)
        return;

    std::vector<unsigned short> rgb16f(w * h * 3);

    unsigned short* p = rgb16f.data();
    const ColorRGB* pixel = pixels;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            *p++ = float_to_half(pixel->r);
            *p++ = float_to_half(pixel->g);
            *p++ = float_to_half(pixel->b);
            pixel++;
        }
    }

    size_t exr_size;
    unsigned char* exr_data = miniexr_write(w, h, 3, rgb16f.data(), &exr_size);

    size_t bytes_written = fwrite(exr_data, 1, exr_size, file);
    ASSERT(bytes_written == exr_size);

    free(exr_data);
    fclose(file);
}
