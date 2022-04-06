// ======================================================================== //
// Copyright 2019 Ingo Wald                                                 //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include "SemanticParser.h"

#ifndef PRINT
# define PRINT(var) std::cout << #var << "=" << var << std::endl;
# define PING std::cout << __FILE__ << "::" << __LINE__ << ": " << __PRETTY_FUNCTION__ << std::endl;
#endif

namespace pbrt {

  Material::SP SemanticParser::createMaterial_hair(pbrt::syntactic::Material::SP in)
  {
    HairMaterial::SP mat = std::make_shared<HairMaterial>(in->name);
    for (auto it : in->param) {
      std::string name = it.first;
      if (name == "eumelanin") {
        mat->eumelanin = in->getParam1f(name);
      } else if (name == "alpha") {
        mat->alpha = in->getParam1f(name);
      } else if (name == "beta_m") {
        mat->beta_m = in->getParam1f(name);
      } else if (name == "type") {
        /* ignore */
      } else
        throw std::runtime_error("as-yet-unhandled hair-material parameter '"+it.first+"'");
    };
    return mat;
  }

  Material::SP SemanticParser::createMaterial_uber(pbrt::syntactic::Material::SP in)
  {
    UberMaterial::SP mat = std::make_shared<UberMaterial>(in->name);
    for (auto it : in->param) {
      std::string name = it.first;
      if (name == "Kd") {
        if (in->hasParamTexture(name)) {
          mat->kd = vec3f(1.f);
          mat->map_kd = findOrCreateTexture(in->getParamTexture(name));
        } else
          in->getParam3f(&mat->kd.x,name);
      }
      else if (name == "Kr") {
        if (in->hasParamTexture(name)) {
          mat->kr = vec3f(1.f);
          mat->map_kr = findOrCreateTexture(in->getParamTexture(name));
        } else
          in->getParam3f(&mat->kr.x,name);
      }
      else if (name == "Kt") {
        if (in->hasParamTexture(name)) {
          mat->kt = vec3f(1.f);
          mat->map_kt = findOrCreateTexture(in->getParamTexture(name));
        } else
          in->getParam3f(&mat->kt.x,name);
      }
      else if (name == "Ks") {
        if (in->hasParamTexture(name)) {
          mat->ks = vec3f(1.f);
          mat->map_ks = findOrCreateTexture(in->getParamTexture(name));
        } else
          in->getParam3f(&mat->ks.x,name);
      }
      else if (name == "alpha") {
        if (in->hasParamTexture(name)) {
          mat->alpha = 1.f;
          mat->map_alpha = findOrCreateTexture(in->getParamTexture(name));
        } else
          mat->alpha = in->getParam1f(name);
      }
      else if (name == "opacity") {
        if (in->hasParamTexture(name)) {
          mat->opacity = vec3f(1.f);
          mat->map_opacity = findOrCreateTexture(in->getParamTexture(name));
        } else
          in->getParam3f(&mat->opacity.x,name);
      }
      else if (name == "index") {
        mat->index = in->getParam1f(name);
      }
      else if (name == "roughness") {
        if (in->hasParamTexture(name))
          mat->map_roughness = findOrCreateTexture(in->getParamTexture(name));
        else if (in->hasParam1f(name))
          mat->roughness = in->getParam1f(name);
        else
          throw std::runtime_error("uber::roughness in un-recognized format...");
        // else
        //   in->getParam3f(&mat->roughness.x,name);
      }
      else if (name == "uroughness") {
        // if (in->hasParamTexture(name))
        //   mat->map_uRoughness = findOrCreateTexture(in->getParamTexture(name));
        // else
          mat->uRoughness = in->getParam1f(name);
      }
      else if (name == "vroughness") {
        // if (in->hasParamTexture(name))
        //   mat->map_vRoughness = findOrCreateTexture(in->getParamTexture(name));
        // else
          mat->vRoughness = in->getParam1f(name);
      }
      // else if (name == "remaproughness") {
      //   mat->remapRoughness = in->getParamBool(name);
      // }
      else if (name == "shadowalpha") {
        if (in->hasParamTexture(name)) {
          mat->shadowAlpha = 1.f;
          mat->map_shadowAlpha = findOrCreateTexture(in->getParamTexture(name));
        } else
          mat->shadowAlpha = in->getParam1f(name);
      }
      else if (name == "bumpmap") {
        mat->map_bump = findOrCreateTexture(in->getParamTexture(name));
      }
      else if (name == "type") {
        /* ignore */
      } else
        throw std::runtime_error("un-handled uber-material parameter '"+it.first+"'");
    };
    return mat;
  }

  //
  // According to PBRT V3, if eta or k parameters are not specified then 
  // we use copper's parameters as default.
  // The following spectrum values are from the PBRT v3 source code.
  //
  const int CopperSamples = 56;
  static const float CopperWavelengths[CopperSamples] = {
      298.7570554, 302.4004341, 306.1337728, 309.960445,  313.8839949,
      317.9081487, 322.036826,  326.2741526, 330.6244747, 335.092373,
      339.6826795, 344.4004944, 349.2512056, 354.2405086, 359.374429,
      364.6593471, 370.1020239, 375.7096303, 381.4897785, 387.4505563,
      393.6005651, 399.9489613, 406.5055016, 413.2805933, 420.2853492,
      427.5316483, 435.0322035, 442.8006357, 450.8515564, 459.2006593,
      467.8648226, 476.8622231, 486.2124627, 495.936712,  506.0578694,
      516.6007417, 527.5922468, 539.0616435, 551.0407911, 563.5644455,
      576.6705953, 590.4008476, 604.8008683, 619.92089,   635.8162974,
      652.5483053, 670.1847459, 688.8009889, 708.4810171, 729.3186941,
      751.4192606, 774.9011125, 799.8979226, 826.5611867, 855.0632966,
      885.6012714};

  static const float CopperN[CopperSamples] = {
      1.400313, 1.38,  1.358438, 1.34,  1.329063, 1.325, 1.3325,   1.34,
      1.334375, 1.325, 1.317812, 1.31,  1.300313, 1.29,  1.281563, 1.27,
      1.249062, 1.225, 1.2,      1.18,  1.174375, 1.175, 1.1775,   1.18,
      1.178125, 1.175, 1.172812, 1.17,  1.165312, 1.16,  1.155312, 1.15,
      1.142812, 1.135, 1.131562, 1.12,  1.092437, 1.04,  0.950375, 0.826,
      0.645875, 0.468, 0.35125,  0.272, 0.230813, 0.214, 0.20925,  0.213,
      0.21625,  0.223, 0.2365,   0.25,  0.254188, 0.26,  0.28,     0.3};

  static const float CopperK[CopperSamples] = {
      1.662125, 1.687, 1.703313, 1.72,  1.744563, 1.77,  1.791625, 1.81,
      1.822125, 1.834, 1.85175,  1.872, 1.89425,  1.916, 1.931688, 1.95,
      1.972438, 2.015, 2.121562, 2.21,  2.177188, 2.13,  2.160063, 2.21,
      2.249938, 2.289, 2.326,    2.362, 2.397625, 2.433, 2.469187, 2.504,
      2.535875, 2.564, 2.589625, 2.605, 2.595562, 2.583, 2.5765,   2.599,
      2.678062, 2.809, 3.01075,  3.24,  3.458187, 3.67,  3.863125, 4.05,
      4.239563, 4.43,  4.619563, 4.817, 5.034125, 5.26,  5.485625, 5.717};

  static const Spectrum copper_eta = []() {
      Spectrum s;
      for (int i = 0; i < CopperSamples; i++)
          s.spd.push_back({ CopperWavelengths[i], CopperN[i] });
      return s;
  }();

  static const Spectrum copper_k = []() {
      Spectrum s;
      for (int i = 0; i < CopperSamples; i++)
          s.spd.push_back({ CopperWavelengths[i], CopperK[i] });
      return s;
  }();

  Material::SP SemanticParser::createMaterial_metal(pbrt::syntactic::Material::SP in)
  {
    bool eta_found = false;
    bool k_found = false;
    MetalMaterial::SP mat = std::make_shared<MetalMaterial>(in->name);
    for (auto it : in->param) {
      std::string name = it.first;
      if (name == "roughness") {
        if (in->hasParamTexture(name))
          mat->map_roughness = findOrCreateTexture(in->getParamTexture(name));
        else
          mat->roughness = in->getParam1f(name);
      }
      else if (name == "uroughness") {
        if (in->hasParamTexture(name))
          mat->map_uRoughness = findOrCreateTexture(in->getParamTexture(name));
        else
          mat->uRoughness = in->getParam1f(name);
      }
      else if (name == "vroughness") {
        if (in->hasParamTexture(name))
          mat->map_vRoughness = findOrCreateTexture(in->getParamTexture(name));
        else
          mat->vRoughness = in->getParam1f(name);
      }
      else if (name == "remaproughness") {
        mat->remapRoughness = in->getParamBool(name);
      }
      else if (name == "eta") {
        if (in->hasParam3f(name))
          in->getParam3f(&mat->eta.x,name);
        else {
          std::size_t N=0;
          in->getParamPairNf(nullptr,&N,name);
          mat->spectrum_eta.spd.resize(N);
          in->getParamPairNf(mat->spectrum_eta.spd.data(),&N,name);
        }
        eta_found = true;
      }
      else if (name == "k") {
        if (in->hasParam3f(name))
          in->getParam3f(&mat->k.x,name);
        else {
          std::size_t N=0;
          in->getParamPairNf(nullptr,&N,name);
          mat->spectrum_k.spd.resize(N);
          in->getParamPairNf(mat->spectrum_k.spd.data(),&N,name);
        }
        k_found = true;
      }
      else if (name == "bumpmap") {
        mat->map_bump = findOrCreateTexture(in->getParamTexture(name));
      }
      else if (name == "type") {
        /* ignore */
      } else
        throw std::runtime_error("un-handled metal-material parameter '"+it.first+"'");
    }
    if (!eta_found) {
        mat->spectrum_eta = copper_eta;
    }
    if (!k_found) {
        mat->spectrum_k = copper_k;
    }
    return mat;
  }

  Material::SP SemanticParser::createMaterial_matte(pbrt::syntactic::Material::SP in)
  {
    MatteMaterial::SP mat = std::make_shared<MatteMaterial>(in->name);
    for (auto it : in->param) {
      std::string name = it.first;
      if (name == "Kd") {
        if (in->hasParamTexture(name)) {
          mat->kd = vec3f(1.f);
          mat->map_kd = findOrCreateTexture(in->getParamTexture(name));
        } else
          in->getParam3f(&mat->kd.x,name);
      }
      else if (name == "sigma") {
        if (in->hasParam1f(name))
          mat->sigma = in->getParam1f(name);
        else 
          mat->map_sigma = findOrCreateTexture(in->getParamTexture(name));
      }
      else if (name == "type") {
        /* ignore */
      }
      else if (name == "bumpmap") {
        mat->map_bump = findOrCreateTexture(in->getParamTexture(name));
      } else
        throw std::runtime_error("un-handled matte-material parameter '"+it.first+"'");
    };
    return mat;
  }

  Material::SP SemanticParser::createMaterial_fourier(pbrt::syntactic::Material::SP in)
  {
    FourierMaterial::SP mat = std::make_shared<FourierMaterial>(in->name);
    for (auto it : in->param) {
      std::string name = it.first;
      if (name == "bsdffile") {
        mat->fileName = in->getParamString(name);
      }
      else if (name == "type") {
        /* ignore */
      } else
        throw std::runtime_error("un-handled fourier-material parameter '"+it.first+"'");
    };
    return mat;
  }

  Material::SP SemanticParser::createMaterial_mirror(pbrt::syntactic::Material::SP in)
  {
    MirrorMaterial::SP mat = std::make_shared<MirrorMaterial>(in->name);
    for (auto it : in->param) {
      std::string name = it.first;
      if (name == "Kr") {
        if (in->hasParamTexture(name)) {
          throw std::runtime_error("mapping Kr for mirror materials not implemented");
        } else
          in->getParam3f(&mat->kr.x,name);
      }
      else if (name == "bumpmap") {
        mat->map_bump = findOrCreateTexture(in->getParamTexture(name));
      }
      else if (name == "type") {
        /* ignore */
      } else
        throw std::runtime_error("un-handled mirror-material parameter '"+it.first+"'");
    };
    return mat;
  }

  Material::SP SemanticParser::createMaterial_substrate(pbrt::syntactic::Material::SP in)
  {
    SubstrateMaterial::SP mat = std::make_shared<SubstrateMaterial>(in->name);
    for (auto it : in->param) {
      std::string name = it.first;
      if (name == "Kd") {
        if (in->hasParamTexture(name)) {
          mat->kd = vec3f(1.f);
          mat->map_kd = findOrCreateTexture(in->getParamTexture(name));
        } else
          in->getParam3f(&mat->kd.x,name);
      }
      else if (name == "Ks") {
        if (in->hasParamTexture(name)) {
          mat->ks = vec3f(1.f);
          mat->map_ks = findOrCreateTexture(in->getParamTexture(name));
        } else
          in->getParam3f(&mat->ks.x,name);
      }
      else if (name == "uroughness") {
        if (in->hasParamTexture(name)) {
          mat->uRoughness = 1.f;
          mat->map_uRoughness = findOrCreateTexture(in->getParamTexture(name));
        } else
          mat->uRoughness = in->getParam1f(name);
      }
      else if (name == "vroughness") {
        if (in->hasParamTexture(name)) {
          mat->vRoughness = 1.f;
          mat->map_vRoughness = findOrCreateTexture(in->getParamTexture(name));
        } else
          mat->vRoughness = in->getParam1f(name);
      }
      else if (name == "remaproughness") {
        mat->remapRoughness = in->getParamBool(name);
      }
      else if (name == "bumpmap") {
        mat->map_bump = findOrCreateTexture(in->getParamTexture(name));
      }
      else if (name == "type") {
        /* ignore */
      } else
        throw std::runtime_error("un-handled substrate-material parameter '"+it.first+"'");
    };
    return mat;
  }

  Material::SP SemanticParser::createMaterial_disney(pbrt::syntactic::Material::SP in)
  {
    DisneyMaterial::SP mat = std::make_shared<DisneyMaterial>(in->name);

    in->getParam3f(&mat->color.x,"color");
    mat->anisotropic    = in->getParam1f("anisotropic",    0.f );
    mat->clearCoat      = in->getParam1f("clearcoat",      0.f );
    mat->clearCoatGloss = in->getParam1f("clearcoatgloss", 1.f );
    mat->diffTrans      = in->getParam1f("difftrans",      1.35f );
    mat->eta            = in->getParam1f("eta",            1.2f );
    mat->flatness       = in->getParam1f("flatness",       0.2f );
    mat->metallic       = in->getParam1f("metallic",       0.f );
    mat->roughness      = in->getParam1f("roughness",      0.9f );
    mat->sheen          = in->getParam1f("sheen",          0.3f );
    mat->sheenTint      = in->getParam1f("sheentint",      0.68f );
    mat->specTrans      = in->getParam1f("spectrans",      0.f );
    mat->specularTint   = in->getParam1f("speculartint",   0.f );
    mat->thin           = in->getParamBool("thin",           true);
    return mat;
  }

  Material::SP SemanticParser::createMaterial_mix(pbrt::syntactic::Material::SP in)
  {
    MixMaterial::SP mat = std::make_shared<MixMaterial>(in->name);
          
    if (in->hasParamTexture("amount"))
      mat->map_amount = findOrCreateTexture(in->getParamTexture("amount"));
    else
      in->getParam3f(&mat->amount.x,"amount");
          
    const std::string name0 = in->getParamString("namedmaterial1");
    if (name0 == "")
      throw std::runtime_error("mix material w/o 'namedmaterial1' parameter");
    const std::string name1 = in->getParamString("namedmaterial2");
    if (name1 == "")
      throw std::runtime_error("mix material w/o 'namedmaterial2' parameter");

    assert(in->attributes);
    pbrt::syntactic::Material::SP mat0 = in->attributes->findNamedMaterial(name0);
    assert(mat0);
    pbrt::syntactic::Material::SP mat1 = in->attributes->findNamedMaterial(name1);
    assert(mat1);

    mat->material0    = findOrCreateMaterial(mat0);
    mat->material1    = findOrCreateMaterial(mat1);
        
    return mat;
  }

  Material::SP SemanticParser::createMaterial_plastic(pbrt::syntactic::Material::SP in)
  {
    PlasticMaterial::SP mat = std::make_shared<PlasticMaterial>(in->name);
    for (auto it : in->param) {
      std::string name = it.first;
      if (name == "Kd") {
        if (in->hasParamTexture(name)) {
          mat->kd = vec3f(1.f);
          mat->map_kd = findOrCreateTexture(in->getParamTexture(name));
        } else
          in->getParam3f(&mat->kd.x,name);
      }
      else if (name == "Ks") {
        if (in->hasParamTexture(name)) {
          mat->ks = vec3f(1.f);
          mat->map_ks = findOrCreateTexture(in->getParamTexture(name));
        } else
          in->getParam3f(&mat->ks.x,name);
      }
      else if (name == "roughness") {
        if (in->hasParamTexture(name))
          mat->map_roughness = findOrCreateTexture(in->getParamTexture(name));
        else
          mat->roughness = in->getParam1f(name);
      }
      else if (name == "remaproughness") {
        mat->remapRoughness = in->getParamBool(name);
      }
      else if (name == "bumpmap") {
        mat->map_bump = findOrCreateTexture(in->getParamTexture(name));
      }
      else if (name == "type") {
        /* ignore */
      } else
        throw std::runtime_error("un-handled plastic-material parameter '"+it.first+"'");
    };
    return mat;
  }
  
  Material::SP SemanticParser::createMaterial_translucent(pbrt::syntactic::Material::SP in)
  {
    TranslucentMaterial::SP mat = std::make_shared<TranslucentMaterial>(in->name);

    in->getParam3f(&mat->transmit.x,"transmit");
    in->getParam3f(&mat->reflect.x,"reflect");
    if (in->hasParamTexture("Kd"))
      mat->map_kd = findOrCreateTexture(in->getParamTexture("Kd"));
    else
      in->getParam3f(&mat->kd.x,"Kd");
          
    return mat;
  }

  Material::SP SemanticParser::createMaterial_glass(pbrt::syntactic::Material::SP in)
  {
    GlassMaterial::SP mat = std::make_shared<GlassMaterial>(in->name);

    in->getParam3f(&mat->kr.x,"Kr");
    in->getParam3f(&mat->kt.x,"Kt");
    mat->index = in->getParam1f("index", 1.5f);
        
    return mat;
  }

      
  /*! do create a track representation of given material, _without_
    checking whether that was already created */
  Material::SP SemanticParser::createMaterialFrom(pbrt::syntactic::Material::SP in)
  {
    if (!in) {
      std::cerr << "warning: empty material!" << std::endl;
      return Material::SP();
    }
      
    const std::string type = in->type=="" ? in->getParamString("type") : in->type;

    // ==================================================================
    if (type == "") 
      return std::make_shared<Material>();
        
    // ==================================================================
    if (type == "plastic") 
      return createMaterial_plastic(in);
      
    // ==================================================================
    if (type == "matte")
      return createMaterial_matte(in);
      
    // ==================================================================
    if (type == "metal") 
      return createMaterial_metal(in);
      
    // ==================================================================
    if (type == "fourier") 
      return createMaterial_fourier(in);
      
    // ==================================================================
    if (type == "mirror") 
      return createMaterial_mirror(in);
      
    // ==================================================================
    if (type == "uber") 
      return createMaterial_uber(in);
      
    // ==================================================================
    if (type == "substrate") 
      return createMaterial_substrate(in);
      
    // ==================================================================
    if (type == "disney") 
      return createMaterial_disney(in);

    // ==================================================================
    if (type == "mix")
      return createMaterial_mix(in);

    // ==================================================================
    if (type == "translucent")
      return createMaterial_translucent(in);

    // ==================================================================
    if (type == "glass") 
      return createMaterial_glass(in);

    // ==================================================================
    if (type == "hair") 
      return createMaterial_hair(in);

    // ==================================================================
#ifndef NDEBUG
    std::cout << "Warning: un-recognizd material type '"+type+"'" << std::endl;
#endif
    return std::make_shared<Material>();
  }

  /*! check if this material has already been imported, and if so,
    find what we imported, and reutrn this. otherwise import and
    store for later use.
      
    important: it is perfectly OK for this material to be a null
    object - the area ligths in moana have this features, for
    example */
  Material::SP SemanticParser::findOrCreateMaterial(pbrt::syntactic::Material::SP in)
  {
    // null materials get passed through ...
    if (!in)
      return Material::SP();

    if (!materialMapping[in]) {
      materialMapping[in] = createMaterialFrom(in);
    }
    return materialMapping[in];
  }


} // ::pbrt
