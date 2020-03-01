#include "std.h"
#include "lib/common.h"
#include "image_texture.h"

#include "lib/image.h"
#include "lib/vector.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

static float lanczos_filter(float x, float radius) {
    x = std::abs(x);
    if (x < 1e-5f)
        return 1.f;
    if (x > radius)
        return 0.f;
    x *= Pi;
    return radius * std::sin(x) * std::sin(x/radius) / (x*x);
}

static void generate_mip_level_with_box_filter(const std::vector<ColorRGB>& src, int width, int height, std::vector<ColorRGB>& dst) {
    // Process one-dimensional image by a separate code path.
    if (width == 1 || height == 1) {
        for (int i = 0; i < int(src.size()); i += 2) {
            dst[i/2] = 0.5f * (src[i] + src[i+1]);
        }
        return;
    }
    // Downsample with box filter.
    ColorRGB* p = dst.data();
    for (int y = 0; y < height; y += 2) {
        const ColorRGB* row0 = &src[y*width];
        const ColorRGB* row0_end = row0 + width;
        const ColorRGB* row1 = row0_end;
        while (row0 != row0_end) {
            *p++ = 0.25f * (row0[0] + row0[1] + row1[0] + row1[1]);
            row0 += 2;
            row1 += 2;
        }
    }
}

void Image_Texture::initialize_from_file(const std::string& image_path, const Image_Texture::Init_Params& params) {
    // Load image data from file.
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

    // Allocate array for mip images.
    int mip_count = 1;
    if (params.generate_mips) {
        uint32_t max_size = uint32_t(std::max(width, height));
        mip_count = log2_int(round_up_to_power_of_2(max_size)) + 1;
    }
    mips.resize(mip_count);

    // Convert image data to floating point representation.
    std::vector<ColorRGB>& texels = mips[0];
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

    // Ensure that base mip level has power of two resolution.
    if (!is_power_of_2(width) || !is_power_of_2(height))
        upsample_base_level_to_power_of_two_resolution();

    // Generate mips if necessary.
    if (params.generate_mips)
        generate_mips();
}

void Image_Texture::upsample_base_level_to_power_of_two_resolution() {
    struct Resample_Weight {
        int first_pixel;
        float pixel_weight[4]; // weights for 4 consecutive pixels covered by the filter kernel
    };
    auto compute_resample_weights = [](int old_resolution, int new_resolution) {
        std::vector<Resample_Weight> rw(new_resolution);

        const float filter_radius = 2.0f;
        const float new_to_old_sample_pos = float(old_resolution) / float(new_resolution);

        // Determine which pixels from the original image contribute to which pixels of the
        // resampled image and compute corresponding weights.
        for (int i = 0; i < new_resolution; i++) {
            float filter_center = (float(i) + 0.5f) * new_to_old_sample_pos;
            rw[i].first_pixel = int(filter_center - filter_radius + 0.5f);
            for (int k = 0; k < 4; k++) {
                float pixel_pos = rw[i].first_pixel + k + 0.5f;
                rw[i].pixel_weight[k] = lanczos_filter(pixel_pos - filter_center, filter_radius);
            }
            // Normalize weights.
            float weight_sum = 0.f;
            for (float w : rw[i].pixel_weight)
                weight_sum += w;
            float inv_weight_sum = 1.f / weight_sum;
            for (float& w : rw[i].pixel_weight)
                w *= inv_weight_sum;
        }
        return rw;
    };

    // Resample in horizontal direction.
    if (!is_power_of_2(width)) {
        int new_width = int(round_up_to_power_of_2(uint32_t(width)));
        const std::vector<Resample_Weight> rw = compute_resample_weights(width, new_width);
        std::vector<ColorRGB> texels(height * new_width);
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < new_width; x++) {
                ColorRGB& t = texels[y*new_width + x];
                for (int k = 0; k < 4; k++) {
                    int src_pixel_x = std::clamp(rw[x].first_pixel + k, 0, width - 1);
                    t += rw[x].pixel_weight[k] * mips[0][y * width + src_pixel_x];
                }
            }
        }
        width = new_width;
        mips[0].swap(texels);
    }

    // Resample in vertical direction.
    if (!is_power_of_2(height)) {
        int new_height = int(round_up_to_power_of_2(uint32_t(height)));
        const std::vector<Resample_Weight> rw = compute_resample_weights(height, new_height);
        std::vector<ColorRGB> texels(new_height * width);
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < new_height; y++) {
                ColorRGB& t = texels[y*width + x];
                for (int k = 0; k < 4; k++) {
                    int src_pixel_y = std::clamp(rw[y].first_pixel + k, 0, height - 1);
                    t += rw[y].pixel_weight[k] * mips[0][src_pixel_y * width + x];
                }
            }
        }
        height = new_height;
        mips[0].swap(texels);
    }

    for (ColorRGB& t : mips[0]) {
        t.r = std::clamp(t.r, 0.f, 1.f);
        t.g = std::clamp(t.g, 0.f, 1.f);
        t.b = std::clamp(t.b, 0.f, 1.f);
    }
}

void Image_Texture::generate_mips() {
    int w = width;
    int h = height;
    for (int i = 1; i < int(mips.size()); i++) {
        const std::vector<ColorRGB>& src = mips[i-1];
        int new_w = std::max(1, w >> 1);
        int new_h = std::max(1, h >> 1);
        mips[i].resize(new_w * new_h);
        generate_mip_level_with_box_filter(mips[i-1], w, h, mips[i]);
        w = new_w;
        h = new_h;
    }
    ASSERT(w == 1 && h == 1);
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

    ColorRGB nearest_texel = mips[0][j * width + i];
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

    const std::vector<ColorRGB>& texels = mips[0];
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
