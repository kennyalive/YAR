// Copyright 2019 Ingo Wald
// SPDX-License-Identifier: Apache-2.0
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
      }
      else if (name == "uroughness") {
          mat->uRoughness = in->getParam1f(name);
          mat->u_roughness_specified = true;
      }
      else if (name == "vroughness") {
          mat->vRoughness = in->getParam1f(name);
          mat->v_roughness_specified = true;
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
      298.7570554f, 302.4004341f, 306.1337728f, 309.960445f,  313.8839949f,
      317.9081487f, 322.036826f,  326.2741526f, 330.6244747f, 335.092373f,
      339.6826795f, 344.4004944f, 349.2512056f, 354.2405086f, 359.374429f,
      364.6593471f, 370.1020239f, 375.7096303f, 381.4897785f, 387.4505563f,
      393.6005651f, 399.9489613f, 406.5055016f, 413.2805933f, 420.2853492f,
      427.5316483f, 435.0322035f, 442.8006357f, 450.8515564f, 459.2006593f,
      467.8648226f, 476.8622231f, 486.2124627f, 495.936712f,  506.0578694f,
      516.6007417f, 527.5922468f, 539.0616435f, 551.0407911f, 563.5644455f,
      576.6705953f, 590.4008476f, 604.8008683f, 619.92089f,   635.8162974f,
      652.5483053f, 670.1847459f, 688.8009889f, 708.4810171f, 729.3186941f,
      751.4192606f, 774.9011125f, 799.8979226f, 826.5611867f, 855.0632966f,
      885.6012714f};

  static const float CopperN[CopperSamples] = {
      1.400313f, 1.38f,  1.358438f, 1.34f,  1.329063f, 1.325f, 1.3325f,   1.34f,
      1.334375f, 1.325f, 1.317812f, 1.31f,  1.300313f, 1.29f,  1.281563f, 1.27f,
      1.249062f, 1.225f, 1.2f,      1.18f,  1.174375f, 1.175f, 1.1775f,   1.18f,
      1.178125f, 1.175f, 1.172812f, 1.17f,  1.165312f, 1.16f,  1.155312f, 1.15f,
      1.142812f, 1.135f, 1.131562f, 1.12f,  1.092437f, 1.04f,  0.950375f, 0.826f,
      0.645875f, 0.468f, 0.35125f,  0.272f, 0.230813f, 0.214f, 0.20925f,  0.213f,
      0.21625f,  0.223f, 0.2365f,   0.25f,  0.254188f, 0.26f,  0.28f,     0.3f};

  static const float CopperK[CopperSamples] = {
      1.662125f, 1.687f, 1.703313f, 1.72f,  1.744563f, 1.77f,  1.791625f, 1.81f,
      1.822125f, 1.834f, 1.85175f,  1.872f, 1.89425f,  1.916f, 1.931688f, 1.95f,
      1.972438f, 2.015f, 2.121562f, 2.21f,  2.177188f, 2.13f,  2.160063f, 2.21f,
      2.249938f, 2.289f, 2.326f,    2.362f, 2.397625f, 2.433f, 2.469187f, 2.504f,
      2.535875f, 2.564f, 2.589625f, 2.605f, 2.595562f, 2.583f, 2.5765f,   2.599f,
      2.678062f, 2.809f, 3.01075f,  3.24f,  3.458187f, 3.67f,  3.863125f, 4.05f,
      4.239563f, 4.43f,  4.619563f, 4.817f, 5.034125f, 5.26f,  5.485625f, 5.717f};

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
          if (in->hasParamTexture(name)) {
              mat->map_uRoughness = findOrCreateTexture(in->getParamTexture(name));
              mat->map_vRoughness = mat->map_uRoughness;
          }
          else {
              mat->uRoughness = in->getParam1f(name);
              mat->vRoughness = mat->uRoughness;
          }
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
    in->getParam3f(&mat->ks.x, "Ks");
    if (in->hasParamTexture("bumpmap")) {
        mat->map_bump = findOrCreateTexture(in->getParamTexture("bumpmap"));
    }
    if (in->hasParam1f("roughness")) {
        mat->roughness = in->getParam1f("roughness");
    }
    return mat;
  }

  Material::SP SemanticParser::createMaterial_glass(pbrt::syntactic::Material::SP in)
  {
    GlassMaterial::SP mat = std::make_shared<GlassMaterial>(in->name);

    in->getParam3f(&mat->kr.x,"Kr");
    in->getParam3f(&mat->kt.x,"Kt");
    mat->index = in->getParam1f("index", 1.5f);
        
    mat->u_roughness = in->getParam1f("uroughness", 0.f);
    mat->v_roughness = in->getParam1f("vroughness", 0.f);
    mat->remap_roughness = in->getParamBool("remaproughness", true);
    if (in->hasParamTexture("bumpmap")) {
        mat->map_bump = findOrCreateTexture(in->getParamTexture("bumpmap"));
    }
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
