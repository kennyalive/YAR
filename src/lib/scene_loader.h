#pragma once

#include "scene.h"

struct String;

// Supported file formats: yar, pbrt, obj
Scene load_scene(const String& input_file);

// Scene loader utilities
int add_scene_texture(const Texture_Descriptor& texture_desc, Scene* scene);
int add_scene_texture(const String& file_name, Scene* scene);
int add_scene_material_parameter(const Parameter& parameter, Scene* scene);
