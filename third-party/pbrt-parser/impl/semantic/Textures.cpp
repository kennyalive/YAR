// Copyright 2019 Ingo Wald
// SPDX-License-Identifier: Apache-2.0

#include "SemanticParser.h"

namespace pbrt {

#ifndef PRINT
# define PRINT(var) std::cout << #var << "=" << var << std::endl;
# define PING std::cout << __FILE__ << "::" << __LINE__ << ": " << __PRETTY_FUNCTION__ << std::endl;
#endif

  /*! extract 'texture' parameters from shape, and assign to shape */
  void SemanticParser::extractTextures(Shape::SP geom, pbrt::syntactic::Shape::SP shape)
  {
    for (auto param : shape->param) {
      if (param.second->getType() != "texture")
        continue;

      geom->textures[param.first] = findOrCreateTexture(shape->getParamTexture(param.first));//param.second);
    }
  }

// From pbrt3 source code
bool HasExtension(const std::string& value, const std::string& ending) {
    if (ending.size() > value.size()) return false;
    return std::equal(
        ending.rbegin(), ending.rend(), value.rbegin(),
        [](char a, char b) { return std::tolower(a) == std::tolower(b); });
}

  Texture::SP SemanticParser::createTexture_image(pbrt::syntactic::Texture::SP in)
  {
    const std::string fileName = in->getParamString("filename");
    if (fileName == "")
      std::cerr << "warning: pbrt image texture, but no filename!?" << std::endl;

    ImageTexture::SP tex = std::make_shared<ImageTexture>(fileName);
    if (in->hasParam1f("scale"))
      tex->scale = in->getParam1f("scale");
    if (in->hasParam1f("uscale"))
      tex->uscale = in->getParam1f("uscale");
    if (in->hasParam1f("vscale"))
      tex->vscale = in->getParam1f("vscale");

    bool defaultGamma = HasExtension(fileName, ".tga") || HasExtension(fileName, ".png");
    tex->gamma = in->getParamBool("gamma", defaultGamma);
    return tex;
  }
  
  Texture::SP SemanticParser::createTexture_mix(pbrt::syntactic::Texture::SP in)
  {
    MixTexture::SP tex = std::make_shared<MixTexture>();

    if (in->hasParam3f("amount"))
      in->getParam3f(&tex->amount.x,"amount");
    else if (in->hasParam1f("amount"))
      tex->amount = vec3f(in->getParam1f("amount"));
    else 
      tex->map_amount = findOrCreateTexture(in->getParamTexture("amount"));
          
    if (in->hasParamTexture("tex1"))
      tex->tex1 = findOrCreateTexture(in->getParamTexture("tex1"));
    else if (in->hasParam3f("tex1"))
      in->getParam3f(&tex->scale1.x,"tex1");
    else
      tex->scale1 = vec3f(in->getParam1f("tex1"));
          
    if (in->hasParamTexture("tex2"))
      tex->tex2 = findOrCreateTexture(in->getParamTexture("tex2"));
    else if (in->hasParam3f("tex2"))
      in->getParam3f(&tex->scale2.x,"tex2");
    else
      tex->scale2 = vec3f(in->getParam1f("tex2"));
    return tex;
  }
  
  Texture::SP SemanticParser::createTexture_scale(pbrt::syntactic::Texture::SP in)
  {
    ScaleTexture::SP tex = std::make_shared<ScaleTexture>();
    if (in->hasParamTexture("tex1")) {
        tex->tex1 = findOrCreateTexture(in->getParamTexture("tex1"));
        if (auto const_texture = std::dynamic_pointer_cast<ConstantTexture>(tex->tex1)) {
            tex->scale1 = const_texture->value;
            tex->tex1 = nullptr;
        }
    }
    else if (in->hasParam3f("tex1"))
      in->getParam3f(&tex->scale1.x,"tex1");
    else
      tex->scale1 = vec3f(in->getParam1f("tex1"));
          
    if (in->hasParamTexture("tex2"))
      tex->tex2 = findOrCreateTexture(in->getParamTexture("tex2"));
    else if (in->hasParam3f("tex2"))
      in->getParam3f(&tex->scale2.x,"tex2");
    else
      tex->scale2 = vec3f(in->getParam1f("tex2"));
    return tex;
  }
  
  Texture::SP SemanticParser::createTexture_ptex(pbrt::syntactic::Texture::SP in)
  {
    const std::string fileName = in->getParamString("filename");
    if (fileName == "")
      std::cerr << "warning: pbrt image texture, but no filename!?" << std::endl;
    return std::make_shared<PtexFileTexture>(fileName);
  }


  Texture::SP SemanticParser::createTexture_constant(pbrt::syntactic::Texture::SP in)
  {
    ConstantTexture::SP tex = std::make_shared<ConstantTexture>();
    if (in->hasParam1f("value"))
      tex->value = vec3f(in->getParam1f("value"));
    else
      in->getParam3f(&tex->value.x,"value");
    return tex;
  }
  
  Texture::SP SemanticParser::createTexture_checker(pbrt::syntactic::Texture::SP in)
  {
    CheckerTexture::SP tex = std::make_shared<CheckerTexture>();
    for (auto it : in->param) {
      const std::string name = it.first;
      if (name == "uscale") {
        tex->uScale = in->getParam1f(name);
        continue;
      }
      if (name == "vscale") {
        tex->vScale = in->getParam1f(name);
        continue;
      }
      if (name == "tex1") {
        in->getParam3f(&tex->tex1.x,name);
        continue;
      }
      if (name == "tex2") {
        in->getParam3f(&tex->tex2.x,name);
        continue;
      }
      throw std::runtime_error("unknown checker texture param '"+name+"'");
    }
    return tex;
  }
  
  /*! do create a track representation of given texture, _without_
    checking whether that was already created */
  Texture::SP SemanticParser::createTextureFrom(pbrt::syntactic::Texture::SP in)
  {
    if (!in) return Texture::SP();
    
    // ------------------------------------------------------------------
    // switch to type-specialized parsing functions ...
    // ------------------------------------------------------------------
    if (in->mapType == "imagemap")
      return createTexture_image(in);
    if (in->mapType == "scale") 
      return createTexture_scale(in);
    if (in->mapType == "mix") 
      return createTexture_mix(in);
    if (in->mapType == "ptex") 
      return createTexture_ptex(in);
    if (in->mapType == "constant") 
      return createTexture_constant(in);
    if (in->mapType == "checkerboard") 
      return createTexture_checker(in);
      
    // ------------------------------------------------------------------
    // do small ones right here (todo: move those to separate
    // functions for cleanliness' sake)
    // ------------------------------------------------------------------
    if (in->mapType == "fbm") {
      FbmTexture::SP tex = std::make_shared<FbmTexture>();
      return tex;
    }
    if (in->mapType == "windy") {
      WindyTexture::SP tex = std::make_shared<WindyTexture>();
      return tex;
    }
    if (in->mapType == "marble") {
      MarbleTexture::SP tex = std::make_shared<MarbleTexture>();
      if (in->hasParam1f("scale"))
        tex->scale = in->getParam1f("scale");
      return tex;
    }
    if (in->mapType == "wrinkled") {
      WrinkledTexture::SP tex = std::make_shared<WrinkledTexture>();
      return tex;
    }
    throw std::runtime_error("un-handled pbrt texture type '"+in->toString()+"'");
    return std::make_shared<Texture>();
  }

  Texture::SP SemanticParser::findOrCreateTexture(pbrt::syntactic::Texture::SP in)
  {
    if (!textureMapping[in]) {
      textureMapping[in] = createTextureFrom(in);
    }
    return textureMapping[in];
  }
    

} // ::pbrt
