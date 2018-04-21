#pragma once

#include "bounding_box.h"
#include "triangle.h"

#include <vector>

struct Indexed_Triangle_Mesh {
  std::vector<Vector> vertices;
  std::vector<std::array<int32_t, 3>> face_indices;

  int32_t get_triangle_count() const {
      return static_cast<int32_t>(face_indices.size());
  }

  int32_t get_vertex_count() const {
      return static_cast<int32_t>(vertices.size());
  }

  Triangle get_triangle(int32_t triangle_index) const {
      Triangle t;
      auto& indices = face_indices[triangle_index];
      t[0] = vertices[indices[0]];
      t[1] = vertices[indices[1]];
      t[2] = vertices[indices[2]];
      return t;
  }

  Bounding_Box get_triangle_bounds(int32_t triangle_index) const;
  Bounding_Box get_bounds() const;
  void print_info() const;
};

struct Simple_Triangle_Mesh {
    std::vector<Triangle> triangles;

    int32_t get_triangle_count() const {
        return static_cast<int32_t>(triangles.size());
    }

    Triangle get_triangle(int32_t triangle_index) const {
        return triangles[triangle_index];
    }

    Bounding_Box get_triangle_bounds(int32_t triangle_index) const;
    Bounding_Box get_bounds() const;
    void print_info() const;

    static Simple_Triangle_Mesh from_indexed_mesh(const Indexed_Triangle_Mesh& mesh);
};

using Triangle_Mesh = Simple_Triangle_Mesh;
//using Triangle_Mesh = Indexed_Triangle_Mesh;

