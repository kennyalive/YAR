#include "std.h"
#include "common.h"
#include "image.h"

#include "color.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

bool write_tga_image(const std::string& file_path, const std::vector<ColorRGB>& pixels, int width, int height) {
    ASSERT(int(pixels.size()) == width * height);
    std::vector<uint8_t> srgb_image(pixels.size() * 3);
    uint8_t* p = srgb_image.data();
    for (const ColorRGB& pixel : pixels) {
        ASSERT(pixel.r >= 0.f && pixel.r <= 1.f);
        ASSERT(pixel.g >= 0.f && pixel.g <= 1.f);
        ASSERT(pixel.b >= 0.f && pixel.b <= 1.f);
        *p++ = uint8_t(255.f * srgb_encode(pixel.r) + 0.5f);
        *p++ = uint8_t(255.f * srgb_encode(pixel.g) + 0.5f);
        *p++ = uint8_t(255.f * srgb_encode(pixel.b) + 0.5f);
    }
    return stbi_write_tga(file_path.c_str(), width, height, 3, srgb_image.data()) != 0;
}
