#pragma once

#include "scene.h"

// Supported file formats: yar, pbrt, obj
Scene load_scene(const std::string& input_file);

// Scene loader utilities
int add_scene_texture(const Texture_Descriptor& texture_desc, Scene* scene);
int add_scene_texture(const std::string& file_name, Scene* scene);
