#include "std.h"
#include "lib/common.h"
#include "triangle_mesh.h"

Triangle_Mesh Triangle_Mesh::from_mesh_data(const Mesh_Data& mesh_data) {
    Triangle_Mesh mesh;
    mesh.vertices.resize(mesh_data.vertices.size());
    mesh.normals.resize(mesh_data.vertices.size());
    mesh.uvs.resize(mesh_data.vertices.size());
    mesh.indices.resize(mesh_data.indices.size());

    for (int i = 0; i < (int)mesh_data.vertices.size(); i++) {
        mesh.vertices[i] = mesh_data.vertices[i].pos;
        mesh.normals[i] = mesh_data.vertices[i].normal;
        mesh.uvs[i] = mesh_data.vertices[i].uv;
    }
    for (int i = 0; i < (int)mesh_data.indices.size(); i++) {
        mesh.indices[i] = mesh_data.indices[i];
    }
    mesh.material = mesh_data.material;
    return mesh;
}

Triangle_Mesh Triangle_Mesh::from_diffuse_rectangular_light(const Diffuse_Rectangular_Light& light, int light_index) {
    Triangle_Mesh mesh;

    mesh.vertices.resize(4);
    float x = light.size.x / 2.0f;
    float y = light.size.y / 2.0f;
    mesh.vertices[0] = transform_point(light.light_to_world_transform, Vector3(-x, -y, 0.f));
    mesh.vertices[1] = transform_point(light.light_to_world_transform, Vector3( x, -y, 0.f));
    mesh.vertices[2] = transform_point(light.light_to_world_transform, Vector3( x,  y, 0.f));
    mesh.vertices[3] = transform_point(light.light_to_world_transform, Vector3(-x,  y, 0.f));

    mesh.normals.resize(4);
    Vector3 n = light.light_to_world_transform.get_column(2);
    std::fill(mesh.normals.begin(), mesh.normals.end(), n);

    mesh.indices = { 0, 1, 2, 0, 2, 3};
    mesh.area_light = {Light_Type::diffuse_rectangular, light_index};

    return mesh;
}

Bounding_Box Triangle_Mesh::get_triangle_bounds(uint32_t triangle_index) const {
    const uint32_t* pi = &indices[triangle_index * 3];
    auto bounds = Bounding_Box(vertices[pi[0]]);
    bounds.add_point(vertices[pi[1]]);
    bounds.add_point(vertices[pi[2]]);
    return bounds;
}

Bounding_Box Triangle_Mesh::get_bounds() const {
    Bounding_Box bounds;
    for (uint32_t i = 0; i < get_triangle_count(); i++) {
        bounds = Bounding_Box::get_union(bounds, get_triangle_bounds(i));
    }
    return bounds;
}

void Triangle_Mesh::print_info() const {
    size_t mesh_size =
        vertices.size() * sizeof(Vector3) +
        normals.size() * sizeof(Vector3) +
        uvs.size() * sizeof(Vector2) +
        indices.size() * sizeof(uint32_t);

    printf("[mesh]\n");
    printf("vertex count = %d\n", get_vertex_count());
    printf("triangle count = %d\n", get_triangle_count());
    printf("mesh size = %zdK\n", mesh_size / 1024);
    printf("\n");
}
