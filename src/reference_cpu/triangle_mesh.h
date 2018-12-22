#pragma once

#include "lib/bounding_box.h"
#include "lib/mesh.h"
#include <vector>

struct Triangle_Mesh {
  std::vector<Vector3> vertices;
  std::vector<Vector2> texcoords;
  std::vector<int32_t> face_indices;
  Vector3 k_diffuse;
  Vector3 k_specular;

  static Triangle_Mesh from_mesh_data(const Mesh_Data& mesh_data);

  int32_t get_triangle_count() const {
      assert(face_indices.size() % 3 == 0);
      return static_cast<int32_t>(face_indices.size() / 3);
  }

  int32_t get_vertex_count() const {
      return static_cast<int32_t>(vertices.size());
  }

  void get_triangle(int32_t triangle_index, Vector3& p0, Vector3& p1, Vector3& p2) const {
      const int32_t* indices = &face_indices[triangle_index * 3];
      p0 = vertices[indices[0]];
      p1 = vertices[indices[1]];
      p2 = vertices[indices[2]];
  }

  Bounding_Box get_triangle_bounds(int32_t triangle_index) const;
  Bounding_Box get_bounds() const;
  void print_info() const;
};
