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

// Low-pass filter that tries to remove frequencies higher than sampling_frequency/2. According to
// sampling theory, if we sample the filtered result with rate >= sampling_frequency then we'll get
// good results.
static float lanczos_filter_for_sampling_frequency(float x, float radius, float sampling_frequency) {
    x = std::abs(x);
    if (x < 1e-5f)
        return 1.f;

    // Take into account sampling frequency. Unparameterized sync(x) filter (or sinc(1*x)) removes
    // frequencies higher than 0.5 and allows to sample a signal with a frequency 0.5*2 = 1. It's
    // easy to show that arbitrary sampling frequency can be taken into account if we replace
    // sinc(x) with sinc(sampling_frequency * x). For example, the 1st mip level is sampled with
    // frequency 0.5, so we use sinc(0.5 * x) - regular sinc stretched 2x along x axis.
    x *= sampling_frequency;

    if (x > radius)
        return 0.f;
    x *= Pi;
    return radius * std::sin(x) * std::sin(x/radius) / (x*x);
}

static void generate_next_mip_level_with_box_filter(const std::vector<ColorRGB>& src, int src_width, int src_height, std::vector<ColorRGB>& dst) {
    // Process one-dimensional image by a separate code path.
    if (src_width == 1 || src_height == 1) {
        for (int i = 0; i < int(src.size()); i += 2) {
            dst[i/2] = 0.5f * (src[i] + src[i+1]);
        }
        return;
    }
    // Downsample with box filter.
    ColorRGB* p = dst.data();
    for (int y = 0; y < src_height; y += 2) {
        const ColorRGB* row0 = &src[y*src_width];
        const ColorRGB* row0_end = row0 + src_width;
        const ColorRGB* row1 = row0_end;
        while (row0 != row0_end) {
            *p++ = 0.25f * (row0[0] + row0[1] + row1[0] + row1[1]);
            row0 += 2;
            row1 += 2;
        }
    }
}

// NOTE: For filters with radius >= 2px mipmap generation algorithm should take into account texture
// wrap mode. The drawback of such correct implementation is a dependency between the texture's
// content and the texture addressing mode. Current implementation assumes clamp to edge texture
// addressing mode and will produce slightly incorrect pixels on the edges for samplers that use
// non-clamp addressing.
static std::vector<ColorRGB> generate_mip_level_with_separable_filter(
    const std::vector<ColorRGB>& image, int width, int height,
    int mip_level_to_generate, float filter_radius)
{
    ASSERT(mip_level_to_generate >= 1);
    const int mip_width = std::max(1, width >> mip_level_to_generate);
    const int mip_height = std::max(1, height >> mip_level_to_generate);

    // Texels from the mip levels >= 1 are placed on the integer grid (for the base level it's a
    // half-integer grid). That's why it's fine to *round down* the filter's pixel footprint. For
    // example, a filter with radius 2.3 centered at integer coordinate M can't reach the third
    // pixel to the right which is located at (M + 2.5).
    const int filter_pixel_count = int(filter_radius) * (1 << (mip_level_to_generate - 1));

    std::vector<float> weights(filter_pixel_count);
    {
        const float sampling_frequency = std::pow(0.5f, float(mip_level_to_generate));
        float sum = 0.f;
        for (int i = 0; i < filter_pixel_count; i++) {
            weights[i] = lanczos_filter_for_sampling_frequency(float(i) + 0.5f, filter_radius, sampling_frequency);
            sum += weights[i];
        }
        sum *= 2.f; // take into account that computed weights are only half of the symmetrical filter
        for (float& w : weights) w /= sum; // normalize weights
    }

    // Downsample in horizontal direction.
    std::vector<ColorRGB> texels(mip_width * height);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < mip_width; x++) {
            ColorRGB& t = texels[y*mip_width + x];
            float filter_center = (float(x) + 0.5f) * (width / mip_width);
            // add contribution from left texels
            {
                int src_x = int(filter_center - 0.5f);
                const ColorRGB* src_texel = &image[y*width + src_x];
                for (int k = 0; k < filter_pixel_count; k++) {
                    t += weights[k] * (*src_texel);
                    if (--src_x >= 0)
                        src_texel--;
                }
            }
            // add contribution from right texels
            {
                int src_x = int(filter_center + 0.5f);
                const ColorRGB* src_texel = &image[y*width + src_x];
                for (int k = 0; k < filter_pixel_count; k++) {
                    t += weights[k] * (*src_texel);
                    if (++src_x < width)
                        src_texel++;
                }
            }
        }
    }

    // Downsample in vertical direction.
    std::vector<ColorRGB> result(mip_width * mip_height);
    for (int y = 0; y < mip_height; y++) {
        for (int x = 0; x < mip_width; x++) {
            ColorRGB& t = result[y*mip_width + x];
            float filter_center = (float(y) + 0.5f) * (height / mip_height);
            // add contribution from above texels
            {
                int src_y = int(filter_center - 0.5f);
                const ColorRGB* src_texel = &texels[src_y*mip_width + x];
                for (int k = 0; k < filter_pixel_count; k++) {
                    t += weights[k] * (*src_texel);
                    if (--src_y >= 0)
                        src_texel -= mip_width;
                }
            }
            // add contribution from below texels
            {
                int src_y = int(filter_center + 0.5f);
                const ColorRGB* src_texel = &texels[src_y*mip_width + x];
                for (int k = 0; k < filter_pixel_count; k++) {
                    t += weights[k] * (*src_texel);
                    if (++src_y < height)
                        src_texel += mip_width;
                }
            }
        }
    }
    for (ColorRGB& p : result) {
        p.r = std::clamp(p.r, 0.f, 1.f);
        p.g = std::clamp(p.g, 0.f, 1.f);
        p.b = std::clamp(p.b, 0.f, 1.f);
    }
    return result;
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
        if (i <= 4)
            mips[i] = generate_mip_level_with_separable_filter(mips[0], width, height, i, 3.f);
        else
            generate_next_mip_level_with_box_filter(mips[i-1], w, h, mips[i]);
        //write_tga_image("animal_lanczos3_mip_" + std::to_string(i) + ".tga", mips[i], new_w, new_h);
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
