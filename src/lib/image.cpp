#include "std.h"
#include "common.h"
#include "image.h"

#include "color.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#endif

#define TINYEXR_IMPLEMENTATION
#include "tiny/tinyexr.h"

Image::Image(int width, int height)
    : width(width)
    , height(height)
    , data(width * height)
{
}

bool Image::load_from_file(const std::string file_path, bool decode_srgb, bool* is_hdr_image) {
    if (is_hdr_image)
        *is_hdr_image = false;
    if (get_extension(file_path) == ".exr") {
        // Load image using TinyEXR library.
        float* out;
        int ret = LoadEXR(&out, &width, &height, file_path.c_str(), nullptr);
        if (ret != TINYEXR_SUCCESS)
            return false;
        data.resize(width * height);
        for (int i = 0; i < width * height; i++) {
            data[i] = ColorRGB(&out[i * 4]); // read rgb and ignore alpha
        }
        free(out);
        if (is_hdr_image)
            *is_hdr_image = true;
    }
    else {
        // Load image using STB library.
        stbi_uc* rgba_texels = nullptr;
        {
            int component_count;
            rgba_texels = stbi_load(file_path.c_str(), &width, &height, &component_count, STBI_rgb_alpha);
            if (rgba_texels == nullptr)
                return false;
        }

        // Convert image data to floating-point representation.
        data.resize(width * height);
        {
            const stbi_uc* src = rgba_texels;
            for (int i = 0; i < width * height; i++, src += 4) {
                data[i] = ColorRGB(src[0], src[1], src[2]) * (1.f / 255.f);
                if (decode_srgb) {
                    data[i].r = srgb_decode(data[i].r);
                    data[i].g = srgb_decode(data[i].g);
                    data[i].b = srgb_decode(data[i].b);
                }
            }
        }
        stbi_image_free(rgba_texels);
    }
    return true;
}

bool Image::write_tga(const std::string& file_path) const {
    ASSERT(int(data.size()) == width * height);
    std::vector<uint8_t> srgb_image(data.size() * 3);
    uint8_t* p = srgb_image.data();

    for (const ColorRGB& pixel : data) {
        ASSERT(pixel.r >= 0.f && pixel.r <= 1.f);
        ASSERT(pixel.g >= 0.f && pixel.g <= 1.f);
        ASSERT(pixel.b >= 0.f && pixel.b <= 1.f);

        *p++ = uint8_t(255.f * srgb_encode(pixel.r) + 0.5f);
        *p++ = uint8_t(255.f * srgb_encode(pixel.g) + 0.5f);
        *p++ = uint8_t(255.f * srgb_encode(pixel.b) + 0.5f);
    }
    return stbi_write_tga(file_path.c_str(), width, height, 3, srgb_image.data()) != 0;
}

std::vector<float> Image::get_luminance() const {
    std::vector<float> luminance(data.size());

    for (auto[i, c] : enumerate(data))
        luminance[i] = sRGB_to_XYZ(c)[1];

    return luminance;
}
