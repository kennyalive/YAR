#pragma once

#include "scene.h"

// Supported file formats: yar, pbrt, obj
Scene load_scene(const std::string& input_file);
