#include "camera.h"
#include "triangle.h"
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

void test_triangle_intersection();

int main() {
    const int w = 1280;
    const int h = 720;

    Matrix3x4 camera_to_world;
    memset(&camera_to_world, 0, sizeof(Matrix3x4));
    camera_to_world.a[0][0] = 1;
    camera_to_world.a[2][1] = -1;
    camera_to_world.a[1][2] = 1;

    Camera camera(camera_to_world, Vector2(float(w), float(h)), 60.f);

    Triangle tri;
    tri[0] = Vector(-2, 5, -1);
    tri[1] = Vector( 2, 5, -1);
    tri[2] = Vector( 0, 5,  2);

    std::vector<Vector> image(w * h);

    Vector* pixel = image.data();
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            Ray ray = camera.generate_ray(Vector2(j + 0.5f, i + 0.5f));
            *pixel = Vector(0);
            Triangle_Intersection hit;
            if (intersect_triangle(ray, tri, hit)) {
                *pixel = Vector(1.f);
            }

            pixel++;
        }
    }

    write_ppm_image("image.ppm", image.data(), w, h);

    //test_triangle_intersection();
    return 0;
}
