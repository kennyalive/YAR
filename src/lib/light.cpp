#include "std.h"
#include "common.h"
#include "light.h"

Triangle_Mesh Diffuse_Rectangular_Light::get_geometry() const {
    Triangle_Mesh mesh;

    mesh.vertices.resize(4);
    float x = size.x / 2.0f;
    float y = size.y / 2.0f;
    mesh.vertices[0] = transform_point(light_to_world_transform, Vector3(-x, -y, 0.f));
    mesh.vertices[1] = transform_point(light_to_world_transform, Vector3( x, -y, 0.f));
    mesh.vertices[2] = transform_point(light_to_world_transform, Vector3( x,  y, 0.f));
    mesh.vertices[3] = transform_point(light_to_world_transform, Vector3(-x,  y, 0.f));

    mesh.normals.resize(4);
    Vector3 n = light_to_world_transform.get_column(2);
    std::fill(mesh.normals.begin(), mesh.normals.end(), n);

    mesh.uvs.resize(4);
    std::fill(mesh.uvs.begin(), mesh.uvs.end(), Vector2{0, 0});

    mesh.indices = { 0, 1, 2, 0, 2, 3};
    return mesh;
}

void Lights::append(const Lights& lights) {
    #define append_lights(x) x.insert(x.end(), lights.x.begin(), lights.x.end())
    append_lights(point_lights);
    append_lights(directional_lights);
    append_lights(diffuse_rectangular_lights);
    append_lights(diffuse_sphere_lights);
    #undef append_lights

    if (lights.has_environment_light) {
        environment_light = lights.environment_light;
        has_environment_light = true;
    }
}

bool Lights::has_lights() const {
    return
        point_lights.size() ||
        directional_lights.size() ||
        diffuse_rectangular_lights.size() ||
        diffuse_sphere_lights.size() ||
        has_environment_light;
}

void Lights::update_total_light_count() {
    total_light_count =
        (int)point_lights.size() + 
        (int)directional_lights.size() +
        (int)diffuse_rectangular_lights.size() +
        (int)diffuse_sphere_lights.size() +
        has_environment_light;
}
