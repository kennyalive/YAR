#include "camera.h"
#include "colorimetry.h"
#include "common.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "light.h"
#include "spectrum.h"
#include "triangle_mesh.h"
#include "triangle_mesh_loader.h"
#include "vector.h"

#include <cassert>
#include <cstdio>
#include <vector>
#include <unordered_map>

#include "half/half.h"

#include "tiny_obj_loader.h"

void write_ppm_image(const char* file_name, const Vector3* pixels, int w, int h) {
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

void write_exr_image(const char* file_name, const Vector3* pixels, int w, int h) {
    FILE* file;
    if (fopen_s(&file, file_name, "wb") != 0)
        return;

    std::vector<unsigned short> rgb16f(w * h * 3);

    unsigned short* p = rgb16f.data();
    const Vector3* pixel = pixels;
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

struct Scene {
    std::vector<Triangle_Mesh> meshes;
};

namespace {
struct Vertex {
    Vector3 p;
    Vector2 uv;

    bool operator==(const Vertex& other) const {
        return p == other.p && uv == other.uv;
    }
};
}

namespace std {
template<> struct hash<Vertex> {
    size_t operator()(const Vertex& vertex) const {
        size_t hash = 0;
        hash_combine(hash, vertex.p.x);
        hash_combine(hash, vertex.p.y);
        hash_combine(hash, vertex.p.z);
        hash_combine(hash, vertex.uv.x);
        hash_combine(hash, vertex.uv.y);
        return hash;
    }
};}

Scene load_conference_scene() {
    Scene scene;

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, get_resource_path("conference/conference.obj").c_str(), get_resource_path("conference/").c_str())) {
        error("failed to load obj file");
    }

    scene.meshes.resize(shapes.size());

    for (size_t i = 0; i < shapes.size(); i++){
        const tinyobj::shape_t& shape = shapes[i];

        // Should be true since triangulate flag is specified, also assume there are no unused indices at the end of array.
        assert(shape.mesh.num_face_vertices.size() * 3 == shape.mesh.indices.size());

        scene.meshes[i].face_indices.resize(shape.mesh.indices.size());

        std::unordered_map<Vertex, int32_t> unique_vertices;

        for (size_t k = 0; k < shape.mesh.indices.size(); k++) {
            const tinyobj::index_t& index = shape.mesh.indices[k];
            Vertex v;

            const float* p = &attrib.vertices[3 * index.vertex_index];
            v.p = Vector3(p[0], p[1], p[2]);

            if (index.texcoord_index != -1) {
                const float* uv = &attrib.texcoords[2 * index.texcoord_index];
                v.uv = Vector2(uv[0], uv[1]);
            }

            if (unique_vertices.count(v) == 0) {
                unique_vertices[v] = (int32_t)scene.meshes[i].vertices.size();

                scene.meshes[i].vertices.push_back(v.p * 0.003f);
                scene.meshes[i].texcoords.push_back(v.uv);
            }
            scene.meshes[i].face_indices[k] = unique_vertices[v];
        }
    }
    return scene;
}

int run_playground() {
    const int w = 1280;
    const int h = 720;

    Matrix3x4 camera_to_world;
    memset(&camera_to_world, 0, sizeof(Matrix3x4));
  /*  camera_to_world.a[0][0] = 1;
    camera_to_world.a[2][1] = -1;
    camera_to_world.a[1][2] = 1;*/

    camera_to_world.a[0][0] = -1;

    camera_to_world.a[0][1] = 0;
    camera_to_world.a[1][1] = -0.6f;
    camera_to_world.a[2][1] = -0.8f;

    camera_to_world.a[0][2] = 0;
    camera_to_world.a[1][2] = -0.8f;
    camera_to_world.a[2][2] = 0.6f;

    camera_to_world.a[0][3] = 1;
    camera_to_world.a[1][3] = 1.8f;
    camera_to_world.a[2][3] = -0.5f;

    Camera camera(camera_to_world, Vector2(float(w), float(h)), 60.f);

    Scene scene = load_conference_scene();

    std::vector<Mesh_KdTree> kdtrees(scene.meshes.size());

    for (size_t i = 0; i < scene.meshes.size(); i++) {
        Timestamp t;
        kdtrees[i] = build_kdtree(scene.meshes[i]);
        int time = int(elapsed_milliseconds(t));
        printf("KdTree %zd build time = %dms\n", i, time);

    }

    printf("conference scene processed\n");

    TwoLevel_KdTree kdtree = build_kdtree(kdtrees);
    //Mesh_KdTree kdtree = std::move(kdtrees[38]);

    printf("two-level tree created\n");

  /*  Simple_Triangle_Mesh mesh = Simple_Triangle_Mesh::from_indexed_mesh(*LoadTriangleMesh("data/teapot.stl"));
    Mesh_KdTree kdtree = load_mesh_kdtree("data/teapot.kdtree", mesh);*/

    // Uniform spectrum that produces luminous flux of 1600Lm.
    float P = 1600 * 800; // Lm
    float C = P / (683.f * CIE_Y_integral); // [W/m]
    Sampled_Spectrum s = Sampled_Spectrum::constant_spectrum(C);

    XYZ xyz = s.emission_spectrum_to_XYZ();

    Point_Light light;
    light.intensity = RGB(xyz);
    light.world_position = Vector3(0, -50, 10);

    float albedo = 1.0f;

    std::vector<Vector3> image(w * h);


    Timestamp t;
    Vector3* pixel = image.data();
    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            Ray ray = camera.generate_ray(Vector2(j + 0.5f, i + 0.5f));
            *pixel = Vector3(0);
            Local_Geometry local_geom;
            if (kdtree.intersect(ray, local_geom) != Infinity) {
                Vector3 light_dir = (light.world_position - local_geom.position).normalized();
                float dd = (light.world_position - local_geom.position).squared_length();
                RGB L = light.intensity * (albedo / (Pi * dd) * dot(local_geom.normal, light_dir));
                *pixel = Vector3(L[0], L[1], L[2]);
                //*pixel = Vector3(100, 100, 100);
            }

            pixel++;
        }
    }

    int time = int(elapsed_milliseconds(t));
    printf("image rendered in %d ms\n", time);

    write_ppm_image("image.ppm", image.data(), w, h);
    write_exr_image("image.exr", image.data(), w, h);

    //test_triangle_intersection();
    //test_kdtree();
    return 0;
}
