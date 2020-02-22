#include "std.h"
#include "lib/common.h"
#include "texture.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

void Texture::init_from_file(const std::string& image_path) {
    int component_count;

    stbi_uc* rgba_texels = stbi_load(image_path.c_str(), &width, &height, &component_count, STBI_rgb_alpha);
    if (rgba_texels == nullptr)
        error("failed to load image file: %s", image_path.c_str());

    texels.resize(width * height);
    const stbi_uc* texel = rgba_texels;
    for (int i = 0; i < width*height; i++, texel += 4) {
        texels[i] = ColorRGB(texel[0], texel[1], texel[2]) * (1.f/255.f);
    }
    stbi_image_free(rgba_texels);
}
