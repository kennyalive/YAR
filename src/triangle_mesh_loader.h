#pragma once

#include <memory>
#include <string>

struct Triangle_Mesh;
std::unique_ptr<Triangle_Mesh> LoadTriangleMesh(const std::string& fileName);
