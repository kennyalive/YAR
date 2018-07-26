#include "camera.h"
#include "colorimetry.h"
#include "common.h"
#include "intersection.h"
#include "kdtree.h"
#include "light.h"
#include "spectrum.h"
#include "triangle_mesh.h"
#include "triangle_mesh_loader.h"
#include "vector.h"

#include <cassert>
#include <cstdio>
#include <vector>

#include "3rdparty/half/half.h"

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

unsigned char* miniexr_write(unsigned width, unsigned height, unsigned channels, const void* rgba16f, size_t* outSize);

void write_exr_image(const char* file_name, const Vector* pixels, int w, int h) {
    FILE* file;
    if (fopen_s(&file, file_name, "wb") != 0)
        return;

    std::vector<unsigned short> rgb16f(w * h * 3);

    unsigned short* p = rgb16f.data();
    const Vector* pixel = pixels;
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            *p++ = float_to_half(pixel->x);
            *p++ = float_to_half(pixel->y);
            *p++ = float_to_half(pixel->z);
            pixel++;
        }
    }

    size_t exr_size;
    unsigned char* exr_data = miniexr_write(w, h, 3, rgb16f.data(), &exr_size);

    size_t bytes_written = fwrite(exr_data, 1, exr_size, file);
    assert(bytes_written == exr_size);

    free(exr_data);
    fclose(file);
}

void test_triangle_intersection();
void test_kdtree();

int main() {
    const int w = 1280;
    const int h = 720;

    Matrix3x4 camera_to_world;
    memset(&camera_to_world, 0, sizeof(Matrix3x4));
    camera_to_world.a[0][0] = 1;
    camera_to_world.a[2][1] = -1;
    camera_to_world.a[1][2] = 1;

    camera_to_world.a[1][3] = -55;
    camera_to_world.a[2][3] =  15;

    Camera camera(camera_to_world, Vector2(float(w), float(h)), 60.f);

    Simple_Triangle_Mesh mesh = Simple_Triangle_Mesh::from_indexed_mesh(*LoadTriangleMesh("teapot.stl"));
    KdTree kdtree("teapot.kdtree", mesh);

    Vector normal(0, -1, 0);

    // Uniform spectrum that produces luminous flux of 1600Lm.
    float P = 1600 * 200; // Lm
    float C = P / (683.f * CIE_Y_integral); // [W/m]
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(C);

    XYZ xyz = s.emission_spectrum_to_XYZ();

    Point_Light light;
    light.intensity = RGB(xyz);
    light.world_position = Vector(0, -50, 10);

    float albedo = 1.0f;

    std::vector<Vector> image(w * h);

    Vector* pixel = image.data();
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            Ray ray = camera.generate_ray(Vector2(j + 0.5f, i + 0.5f));
            *pixel = Vector(0);
            Local_Geometry local_geom;
            if (kdtree.intersect(ray, local_geom) != Infinity) {
                Vector light_dir = (light.world_position - local_geom.position).normalized();
                float dd = (light.world_position - local_geom.position).squared_length();
                RGB L = light.intensity * (albedo / (Pi * dd) * dot(local_geom.normal, light_dir));
                *pixel = Vector(L[0], L[1], L[2]);
            }

            pixel++;
        }
    }

    write_ppm_image("image.ppm", image.data(), w, h);
    write_exr_image("image.exr", image.data(), w, h);

    test_triangle_intersection();
    test_kdtree();
    return 0;
}
