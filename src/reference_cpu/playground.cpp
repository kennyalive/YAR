#include "camera.h"
#include "colorimetry.h"
#include "intersection.h"
#include "kdtree.h"
#include "kdtree_builder.h"
#include "light.h"
#include "spectrum.h"
#include "triangle_mesh.h"

#include "io/io.h"
#include "lib/common.h"
#include "lib/mesh.h"
#include "lib/scene.h"
#include "lib/vector.h"

#include <vector>

void test_triangle_intersection();
void test_kdtree();

int run_playground(const Scene_Data& scene_data, const Matrix3x4& camera_to_world_vk, bool* active) {
    const int w = 1280;
    const int h = 720;

    Matrix3x4 camera_to_world = camera_to_world_vk;
    for (int i = 0; i < 3; i++) {
        float tmp = camera_to_world.a[i][1];
        camera_to_world.a[i][1] = -camera_to_world.a[i][2];
        camera_to_world.a[i][2] = tmp;
    }

    Camera camera(camera_to_world, Vector2(float(w), float(h)), 60.f);

    std::vector<Mesh_KdTree> kdtrees(scene_data.meshes.size());
    std::vector<Triangle_Mesh> meshes(scene_data.meshes.size());

    for (size_t i = 0; i < scene_data.meshes.size(); i++) {
        Timestamp t;
        meshes[i] = Triangle_Mesh::from_mesh_data(scene_data.meshes[i]);
        kdtrees[i] = build_kdtree(meshes[i]);
        int time = int(elapsed_milliseconds(t));
        printf("KdTree %zd build time = %dms\n", i, time);

    }

    printf("conference scene processed\n");
    TwoLevel_KdTree kdtree = build_kdtree(kdtrees);
    printf("two-level tree created\n");
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
                Vector3 light_dir = (scene_data.rgb_point_lights[0].position - local_geom.position).normalized();
                float dd = (scene_data.rgb_point_lights[0].position - local_geom.position).squared_length();
                RGB L = scene_data.rgb_point_lights[0].intensity * (albedo / (Pi * dd) * dot(local_geom.normal, light_dir));
                *pixel = Vector3(L[0], L[1], L[2]);
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
