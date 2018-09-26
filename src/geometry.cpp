#include "geometry.h"
#include "vector.h"
#include <tiny_obj_loader.h>
#include <unordered_map>

namespace std {
template<> struct hash<Vertex> {
    size_t operator()(Vertex const& v) const {
        size_t hash = 0;
        hash_combine(hash, v.pos);
        hash_combine(hash, v.tex_coord);
        //hash_combine(hash, v.normal);
        return hash;
    }
};
}

Model load_obj_model(const std::string& path) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, path.c_str()))
        error("failed to load obj model: " + path);

    Vector model_min(std::numeric_limits<float>::infinity());
    Vector model_max(-std::numeric_limits<float>::infinity());

    Model model;
    std::unordered_map<Vertex, std::size_t> unique_vertices;
    for (const auto& shape : shapes) {
        for (const auto& index : shape.mesh.indices) {
            Vertex vertex;
            vertex.pos = {
                attrib.vertices[3 * index.vertex_index + 0],
                attrib.vertices[3 * index.vertex_index + 1],
                attrib.vertices[3 * index.vertex_index + 2]
            };
            vertex.tex_coord = {
                attrib.texcoords[2 * index.texcoord_index + 0],
                1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
            };
            /*vertex.normal = {
                attrib.normals[3 * index.normal_index + 0],
                attrib.normals[3 * index.normal_index + 1],
                attrib.normals[3 * index.normal_index + 2],
            };*/

            if (unique_vertices.count(vertex) == 0) {
                unique_vertices[vertex] = model.vertices.size();
                model.vertices.push_back(vertex);

                // update model bounds
                model_min.x = std::min(model_min.x, vertex.pos.x);
                model_min.y = std::min(model_min.y, vertex.pos.y);
                model_min.z = std::min(model_min.z, vertex.pos.z);
                model_max.x = std::max(model_max.x, vertex.pos.x);
                model_max.y = std::max(model_max.y, vertex.pos.y);
                model_max.z = std::max(model_max.z, vertex.pos.z);
            }
            model.indices.push_back((uint32_t)unique_vertices[vertex]);
        }
    }

    // center the model
    Vector center = (model_min + model_max) * 0.5f;
    for (auto& v : model.vertices) {
        v.pos -= center;
    }
    return model;
}
