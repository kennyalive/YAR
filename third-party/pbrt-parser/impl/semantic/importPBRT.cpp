// Copyright 2019 Ingo Wald
// SPDX-License-Identifier: Apache-2.0

#include "pbrtParser/Scene.h"
#include "../syntactic/Scene.h"
#include "SemanticParser.h"
// std
#include <map>
#include <sstream>

namespace pbrt {
  Scene::SP importPBRT(const std::string &fileName, const std::string &basePath)
  {
    pbrt::syntactic::Scene::SP pbrt;
    if (endsWith(fileName,".pbrt"))
      pbrt = pbrt::syntactic::Scene::parse(fileName, basePath);
    else
      throw std::runtime_error("could not detect input file format!? (unknown extension in '"+fileName+"')");
      
    Scene::SP scene = SemanticParser(pbrt).result;
    createFilm(scene,pbrt);
    createSampler(scene,pbrt);
    createIntegrator(scene,pbrt);
    createPixelFilter(scene,pbrt);
    for (auto cam : pbrt->cameras)
      scene->cameras.push_back(createCamera(cam));
    return scene;
  }

} // ::pbrt
