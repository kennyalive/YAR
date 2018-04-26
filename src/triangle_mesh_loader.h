#pragma once

#include <memory>
#include <string>

struct Indexed_Triangle_Mesh;
std::unique_ptr<Indexed_Triangle_Mesh> LoadTriangleMesh(const std::string& fileName);
