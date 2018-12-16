#include "camera.h"
#include "colorimetry.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "light.h"
#include "spectrum.h"
#include "triangle_mesh.h"
#include "triangle_mesh_loader.h"

#include "io/io.h"
#include "lib/common.h"
#include "lib/mesh.h"
#include "lib/vector.h"

#include <cassert>
#include <vector>
#include <unordered_map>

void test_triangle_intersection();
void test_kdtree();

int run_playground(const std::vector<Mesh_Data>& mesh_data, const Matrix3x4& camera_to_world_vk, bool* active) {
    const int w = 1280;
    const int h = 720;

    Matrix3x4 camera_to_world = camera_to_world_vk;
    for (int i = 0; i < 3; i++) {
        float tmp = camera_to_world.a[i][1];
        camera_to_world.a[i][1] = -camera_to_world.a[i][2];
        camera_to_world.a[i][2] = tmp;
    }

    Camera camera(camera_to_world, Vector2(float(w), float(h)), 60.f);

    std::vector<Mesh_KdTree> kdtrees(mesh_data.size());
    std::vector<Triangle_Mesh> meshes(mesh_data.size());

    for (size_t i = 0; i < mesh_data.size(); i++) {
        Timestamp t;
        meshes[i] = Triangle_Mesh::from_mesh_data(mesh_data[i]);
        kdtrees[i] = build_kdtree(meshes[i]);
        int time = int(elapsed_milliseconds(t));
        printf("KdTree %zd build time = %dms\n", i, time);

    }

    printf("conference scene processed\n");
    TwoLevel_KdTree kdtree = build_kdtree(kdtrees);
    printf("two-level tree created\n");

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

    write_exr_image("image.exr", image.data(), w, h);

    //test_triangle_intersection();
    //test_kdtree();
    *active = false;
    return 0;
}
