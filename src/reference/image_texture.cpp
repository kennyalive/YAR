#include "std.h"
#include "lib/common.h"
#include "image_texture.h"

#include "lib/vector.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

void Image_Texture::initialize_from_file(const std::string& image_path, const Image_Texture::Init_Params& params) {
    stbi_uc* rgba_texels = nullptr;
    {
        // we need lock because stbi_set_flip_vertically_on_load is not thread safe
        static std::mutex stb_load_mutex;
        std::scoped_lock<std::mutex> lock(stb_load_mutex);

        stbi_set_flip_vertically_on_load(params.flip_vertically);
        int component_count;
        rgba_texels = stbi_load(image_path.c_str(), &width, &height, &component_count, STBI_rgb_alpha);
    }

    if (rgba_texels == nullptr)
        error("failed to load image file: %s", image_path.c_str());

    texels.resize(width * height);
    const stbi_uc* texel = rgba_texels;
    for (int i = 0; i < width*height; i++, texel += 4) {
        texels[i] = ColorRGB(texel[0], texel[1], texel[2]) * (1.f/255.f);
        if (params.decode_srgb) {
            texels[i].r = srgb_decode(texels[i].r);
            texels[i].g = srgb_decode(texels[i].g);
            texels[i].b = srgb_decode(texels[i].b);
        }
    }
    stbi_image_free(rgba_texels);
}

ColorRGB Image_Texture::sample_nearest(const Vector2& uv, Wrap_Mode wrap_mode) const {
    int i = int(uv.x * width);
    int j = int(uv.y * height);

    if (wrap_mode == Wrap_Mode::repeat) {
        i %= width;
        j %= height;
    }
    else {
        ASSERT(wrap_mode == Wrap_Mode::clamp);
        i = std::clamp(i, 0, width-1);
        j = std::clamp(j, 0, height-1);
    }

    ColorRGB nearest_texel = texels[j * width + i];
    return nearest_texel;
}

ColorRGB Image_Texture::sample_bilinear(const Vector2& uv, Wrap_Mode wrap_mode) const {
    float a = uv.x * float(width) - 0.5f;
    float b = uv.y * float(height) - 0.5f;

    float a_floor = std::floor(a);
    float b_floor = std::floor(b);

    int i0 = int(a_floor);
    int j0 = int(b_floor);
    int i1 = i0 + 1;
    int j1 = j0 + 1;

    if (wrap_mode == Wrap_Mode::repeat) {
        // The integer coordinate is additionally incremented before taking remainder
        // in order to handle the case when coordinate's value is -1.
        i0 = (i0 + width) % width;
        j0 = (j0 + height) % height;
        i1 = (i1 + width) % width;
        j1 = (j1 + height) % height;
    }
    else {
        ASSERT(wrap_mode == Wrap_Mode::clamp);
        i0 = std::clamp(i0, 0, width-1);
        j0 = std::clamp(j0, 0, height-1);
        i1 = std::clamp(i1, 0, width-1);
        j1 = std::clamp(j1, 0, height-1);
    }

    ColorRGB texel00 = texels[j0 * width + i0];
    ColorRGB texel01 = texels[j0 * width + i1];
    ColorRGB texel10 = texels[j1 * width + i0];
    ColorRGB texel11 = texels[j1 * width + i1];

    float alpha = a - a_floor;
    float beta = b - b_floor;

    float w_i0 = 1.f - alpha;
    float w_i1 = alpha;
    float w_j0 = 1.f - beta;
    float w_j1 = beta;

    ColorRGB bilinear_texel =
        texel00 * (w_j0 * w_i0) +
        texel01 * (w_j0 * w_i1) +
        texel10 * (w_j1 * w_i0) +
        texel11 * (w_j1 * w_i1);

    return bilinear_texel;
}
