#pragma once

#include "scene.h"

// Supported file formats: yar, pbrt, obj
Scene load_scene(const std::string& input_file);

// Scene loader utilities
int add_scene_texture(const std::string& texture_file_name, Scene* scene);
