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
#include "tinyexr/tinyexr.h"

static std::vector<ColorRGB> load_pfm_image(const std::string& file_path, int* width, int* height) {
    Scoped_File f = fopen(file_path.c_str(), "rb");
    if (!f)
        error("load_pfm_image: failed to open file: %s", file_path.c_str());

    static constexpr int buffer_size = 1024;
    char buffer[buffer_size];

    auto read_ascii_line = [&f, &buffer, &file_path]() {
        int i = 0;
        int ch;
        bool newline_found = false;
        while (i < buffer_size && (ch = fgetc(f)) != EOF) {
            if (ch == '\n') {
                buffer[i++] = 0;
                newline_found = true;
                break;
            }
            buffer[i++] = (char)ch;
        }
        if (!newline_found)
            error("load_pfm_image: header ascii line does not end with a new line character: %s", file_path.c_str());
    };

    // Read file type.
    read_ascii_line();
    if (strncmp(buffer, "PF", 2) != 0)
        error("load_pfm_image: non-RGB file detected, only RGB files are supported: %s", file_path.c_str());

    // Read image dimensions.
    read_ascii_line();
    if (sscanf(buffer, "%d %d", width, height) != 2)
        error("load_pfm_image: failed to read image dimensions: %s", file_path.c_str());

    // Read aspect ratio/endianess value.
    read_ascii_line();
    float endianess;
    if (sscanf(buffer, "%f", &endianess) != 1)
        error("load_pfm_image: failed to read aspect ratio/endianess value: %s", file_path.c_str());
    if (endianess > 0)
        error("load_pfm_image: big endian RGB data is not supported: %s", file_path.c_str());

    // Read RGB floating point triplets.
    int pixel_count = (*width) * (*height);
    std::vector<ColorRGB> pixels(pixel_count);
    if (fread(pixels.data(), sizeof(ColorRGB), pixel_count, f) != pixel_count)
        error("load_pfm_image: failed to read rgb data: %s", file_path.c_str());

    // PFM format defines image rows from bottom to top, we need to flip
    std::vector<ColorRGB> flipped_pixels(pixel_count);
    ColorRGB* dst = flipped_pixels.data();
    ColorRGB* src = pixels.data() + (*width) * (*height - 1);
    for (int i = 0; i < *height; i++) {
        memcpy(dst, src, sizeof(ColorRGB) * (*width));
        dst += *width;
        src -= *width;
    }
    return flipped_pixels;
}

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
    else if (get_extension(file_path) == ".pfm") {
        data = load_pfm_image(file_path, &width, &height);
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
