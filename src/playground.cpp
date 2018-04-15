#include "vector.h"
#include <cstdio>
#include <vector>

void write_ppm_image(const char* file_name, const Vector* pixels, int w, int h) {
    FILE* file;
    if (fopen_s(&file, file_name, "w") != 0)
        return;

    fprintf(file, "P3\n%d %d\n255\n", w, h);

    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            uint8_t r = static_cast<uint8_t>(pixels->x * 255.f);
            uint8_t g = static_cast<uint8_t>(pixels->y * 255.f);
            uint8_t b = static_cast<uint8_t>(pixels->z * 255.f);
            fprintf(file, "%d %d %d\n", r, g, b);
            pixels++;
        }
    }
    fclose(file);
}

int main() {
    const int w = 1280;
    const int h = 720;

    std::vector<Vector> image(w * h);

    Vector* pixel = image.data();
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            *pixel = Vector(static_cast<float>(j)/w, static_cast<float>(i)/h, 0.f);
            pixel++;
        }
    }

    write_ppm_image("image.ppm", image.data(), w, h);
    return 0;
}
