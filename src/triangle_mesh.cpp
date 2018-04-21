#include "triangle_mesh.h"

Bounding_Box Indexed_Triangle_Mesh::get_triangle_bounds(int32_t triangleIndex) const {
  const auto& indices = face_indices[triangleIndex];
  auto bounds = Bounding_Box(vertices[indices[0]]);
  bounds.add_point(vertices[indices[1]]);
  bounds.add_point(vertices[indices[2]]);
  return bounds;
}

Bounding_Box Indexed_Triangle_Mesh::get_bounds() const {
  Bounding_Box bounds;
  for (int32_t i = 0; i < get_triangle_count(); i++) {
    bounds = Bounding_Box::get_union(bounds, get_triangle_bounds(i));
  }
  return bounds;
}

void Indexed_Triangle_Mesh::print_info() const {
    printf("[mesh]\n");
    printf("vertex count = %d\n", get_vertex_count());
    printf("triangle count = %d\n", get_triangle_count());
    size_t vertices_size = get_vertex_count() * sizeof(Vector) / 1024;
    size_t triangles_size = get_triangle_count() * sizeof(face_indices[0]) / 1024;
    printf("mesh size = %zdK\n", vertices_size + triangles_size);
    printf("\n");
}

Simple_Triangle_Mesh Simple_Triangle_Mesh::from_indexed_mesh(const Indexed_Triangle_Mesh& indexed_mesh) {
    Simple_Triangle_Mesh mesh;
    mesh.triangles.resize(indexed_mesh.face_indices.size());
    for (size_t i = 0; i < indexed_mesh.face_indices.size(); i++) {
        const auto& indices = indexed_mesh.face_indices[i];
        for (int k = 0; k < 3; k++) {
            mesh.triangles[i][k] = indexed_mesh.vertices[indices[k]];
        }
    }
    return mesh;
}

Bounding_Box Simple_Triangle_Mesh::get_triangle_bounds(int32_t triangle_index) const {
    const auto& p = triangles[triangle_index];
    auto bounds = Bounding_Box(p[0]);
    bounds.add_point(p[1]);
    bounds.add_point(p[2]);
    return bounds;
}

Bounding_Box Simple_Triangle_Mesh::get_bounds() const {
    Bounding_Box bounds;
    for (int32_t i = 0; i < get_triangle_count(); i++) {
        bounds = Bounding_Box::get_union(bounds, get_triangle_bounds(i));
    }
    return bounds;
}

void Simple_Triangle_Mesh::print_info() const {
    printf("[mesh]\n");
    printf("triangle count = %d\n", get_triangle_count());
    size_t triangles_size = get_triangle_count() * sizeof(triangles[0]) / 1024;
    printf("mesh size = %zdK\n", triangles_size);
    printf("\n");
}
