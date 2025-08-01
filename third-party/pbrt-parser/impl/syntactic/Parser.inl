// ======================================================================== //
// Copyright 2015-2020 Ingo Wald                                            //
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

#include "Lexer.h"
// stl
#include <fstream>
#include <sstream>
#include <stack>
#include <algorithm>
// std
#include <stdio.h>
#include <string.h>

#define _unused(x) ((void)(x))

/*! namespace for all things pbrt parser, both syntactical *and* semantical parser */
namespace pbrt {
  
  /*! namespace for syntactic-only parser - this allows to distringuish
    high-level objects such as shapes from objects or transforms,
    but does *not* make any difference between what types of
    shapes, what their parameters mean, etc. Basically, at this
    level a triangle mesh is nothing but a shape that has a string
    with a given name, and parameters of given names and types */
  namespace syntactic {  
    inline bool operator==(const Token &tk, const std::string &text) { return tk.text == text; }
    inline bool operator==(const Token &tk, const char* text) { return tk.text == text; }
  
    /* split a string on a single character delimiter */
    inline std::vector<std::string> split(const std::string &input, char delim)
    {
      std::stringstream ss(input);
      std::string item;
      std::vector<std::string> elems;
      while (std::getline(ss, item, delim))
        elems.push_back(std::move(item));
      return elems;
    }

    /* split a string on a set of delimiters */
    inline std::vector<std::string> split(const std::string &input, 
                                          const std::string &delim)
    {
      std::vector<std::string> tokens;
      size_t pos = 0;
      while (1) {
        size_t begin = input.find_first_not_of(delim,pos);
        if (begin == input.npos) return tokens;
        size_t end = input.find_first_of(delim,begin);
        tokens.push_back(input.substr(begin,(end==input.npos)?input.npos:(end-begin)));
        pos = end;
      }
    }

 
    template <typename DS>
    inline float BasicParser<DS>::parseFloat()
    {
      Token token = next();
      if (!token)
        throw std::runtime_error("unexpected end of file\n@"+std::string(__PRETTY_FUNCTION__));
      return (float)std::stod(token.text);
    }

    template <typename DS>
    inline vec3f BasicParser<DS>::parseVec3f()
    {
      try {
        const float x = parseFloat();
        const float y = parseFloat();
        const float z = parseFloat();
        return vec3f(x,y,z);
      } catch (std::runtime_error e) {
        throw e.what()+std::string("\n@")+std::string(__PRETTY_FUNCTION__);
      }
    }

    template <typename DS>
    affine3f BasicParser<DS>::parseMatrix()
    {
      const std::string open = next().text;

      assert(open == "[");
      affine3f xfm;
      xfm.l.vx.x = (float)std::stod(next().text.c_str());
      xfm.l.vx.y = (float)std::stod(next().text.c_str());
      xfm.l.vx.z = (float)std::stod(next().text.c_str());
      float vx_w = (float)std::stod(next().text.c_str());
      assert(vx_w == 0.f);
      _unused(vx_w);

      xfm.l.vy.x = (float)std::stod(next().text.c_str());
      xfm.l.vy.y = (float)std::stod(next().text.c_str());
      xfm.l.vy.z = (float)std::stod(next().text.c_str());
      float vy_w = (float)std::stod(next().text.c_str());
      assert(vy_w == 0.f);
      _unused(vy_w);

      xfm.l.vz.x = (float)std::stod(next().text.c_str());
      xfm.l.vz.y = (float)std::stod(next().text.c_str());
      xfm.l.vz.z = (float)std::stod(next().text.c_str());
      float vz_w = (float)std::stod(next().text.c_str());
      assert(vz_w == 0.f);
      _unused(vz_w);

      xfm.p.x    = (float)std::stod(next().text.c_str());
      xfm.p.y    = (float)std::stod(next().text.c_str());
      xfm.p.z    = (float)std::stod(next().text.c_str());
      float p_w  = (float)std::stod(next().text.c_str());
      assert(p_w == 1.f);
      _unused(p_w);

      const std::string close = next().text;
      assert(close == "]");

      return xfm;
    }


    template <typename DS>
    inline std::shared_ptr<Param> BasicParser<DS>::parseParam(std::string &name)
    {
      Token token = peek();

      if (token.type != Token::TOKEN_TYPE_STRING)
        return std::shared_ptr<Param>();

      std::vector<std::string> components = split(next().text,std::string(" \n\t"));

      assert(components.size() == 2);
      std::string type = components[0];
      name = components[1];

      std::shared_ptr<Param> ret; 
      if (type == "float") {
        ret = std::make_shared<ParamArray<float>>(type);
      } else if (type == "color") {
        ret = std::make_shared<ParamArray<float> >(type);
      } else if (type == "blackbody") {
        ret = std::make_shared<ParamArray<float>>(type);
      } else if (type == "rgb") {
        ret = std::make_shared<ParamArray<float> >(type);
      } else if (type == "spectrum") {
        ret = std::make_shared<ParamArray<float>>(type);
      } else if (type == "integer") {
        ret = std::make_shared<ParamArray<int>>(type);
      } else if (type == "bool") {
        ret = std::make_shared<ParamArray<bool>>(type);
      } else if (type == "texture") {
        ret = std::make_shared<ParamArray<Texture>>(type);
      } else if (type == "normal") {
        ret = std::make_shared<ParamArray<float> >(type);
      } else if (type == "point") {
        ret = std::make_shared<ParamArray<float>>(type);
      } else if (type == "point2") {
        ret = std::make_shared<ParamArray<float>>(type);
      } else if (type == "point3") {
        ret = std::make_shared<ParamArray<float>>(type);
      } else if (type == "point4") {
        ret = std::make_shared<ParamArray<float>>(type);
      } else if (type == "vector") {
        ret = std::make_shared<ParamArray<float>>(type);
      } else if (type == "string") {
        ret = std::make_shared<ParamArray<std::string>>(type);
      } else {
        throw std::runtime_error("unknown parameter type '"+type+"' "+token.loc.toString()
                                 +std::string("\n@")+std::string(__PRETTY_FUNCTION__));
      }

      std::string value = next().text;
      if (value == "[") {
        std::string p = next().text;
        
        while (p != "]") {
          if (type == "texture") {
            std::dynamic_pointer_cast<ParamArray<Texture>>(ret)->texture 
              = getTexture(p);
          } else {
            ret->add(p);
          }
          p = next().text;
        }
      } else {
        if (type == "texture") {
          std::dynamic_pointer_cast<ParamArray<Texture>>(ret)->texture 
            = getTexture(value);
        } else if (type == "spectrum") {
          /* parse (wavelength, value) pairs from file */
          std::string includedFileName = value;
          if (includedFileName[0] != '/') {
            includedFileName = rootNamePath+"/"+includedFileName;
          }
          FileType::SP file = std::make_shared<FileType>(includedFileName);
          auto tokens = std::make_shared<BasicLexer<FileType>>(file);
          Token t = tokens->next();
          while (t)
          {
            ret->add(t.text);
            t = tokens->next();
          }
        } else {
          ret->add(value);
        }
      }
      return ret;
    }

    template <typename DS>
    void BasicParser<DS>::parseParams(std::map<std::string, std::shared_ptr<Param> > &params)
    {
      while (1) {
        std::string name;
        std::shared_ptr<Param> param = parseParam(name);
        if (!param) return;
        params[name] = param;
      }
    }

    template <typename DS>
    std::shared_ptr<Texture> BasicParser<DS>::getTexture(const std::string &name) 
    {
      if (currentGraphicsState->findNamedTexture(name) == nullptr)
        {
          std::cerr << "warning: could not find texture named '" << name << "'" << std::endl;
          return std::shared_ptr<Texture> ();
        }
      return currentGraphicsState->findNamedTexture(name);
    }

    template <typename DS>
    BasicParser<DS>::BasicParser(const std::string &basePath) 
      : basePath(basePath)
      , scene(std::make_shared<Scene>())
      , currentGraphicsState(std::make_shared<Attributes>())
    {
      ctm.reset();
      objectStack.push(scene->world);//scene.cast<Object>());
    }

    template <typename DS>
    std::shared_ptr<Object> BasicParser<DS>::getCurrentObject() 
    {
      if (objectStack.empty())
        throw std::runtime_error("no active object!?");
      return objectStack.top(); 
    }

    template <typename DS>
    std::shared_ptr<Object> BasicParser<DS>::findNamedObject(const std::string &name, bool createIfNotExist)
    {
      if (namedObjects.find(name) == namedObjects.end()) {

        if (createIfNotExist) {
          std::shared_ptr<Object> object = std::make_shared<Object>(name);
          namedObjects[name] = object;
        } else {
          throw std::runtime_error("could not find object named '"+name+"'");
        }
      }
      return namedObjects[name];
    }


    template <typename DS>
    void BasicParser<DS>::pushAttributes() 
    {
      Attributes::push(currentGraphicsState);
      materialStack.push(currentMaterial);
      pushTransform();
    }

    template <typename DS>
    void BasicParser<DS>::popAttributes() 
    {
      popTransform();
      Attributes::pop(currentGraphicsState);
      currentMaterial = materialStack.top();
      materialStack.pop();
    }
    
    template <typename DS>
    void BasicParser<DS>::pushTransform() 
    {
      transformStack.push(ctm);
    }

    template <typename DS>
    void BasicParser<DS>::popTransform() 
    {
      ctm = transformStack.top();
      transformStack.pop();
    }
    
    template <typename DS>
    bool BasicParser<DS>::parseTransform(const Token& token)
    {
      if (token == "ActiveTransform") {
        const std::string which = next().text;
        if (which == "All") {
          ctm.startActive = true;
          ctm.endActive = true;
        } else if (which == "StartTime") {
          ctm.startActive = true;
          ctm.endActive = false;
        } else if (which == "EndTime") {
          ctm.startActive = false;
          ctm.endActive = true;
        } else
          throw std::runtime_error("unknown argument '"+which+"' to 'ActiveTransform' command");
          
        return true;
      }
      if (token == "TransformBegin") {
        pushTransform();
        return true;
      }
      if (token == "TransformEnd") {
        popTransform();
        return true;
      }
      if (token == "Scale") {
        vec3f scale = parseVec3f();
        addTransform(affine3f::scale(scale));
        return true;
      }
      if (token == "Translate") {
        vec3f translate = parseVec3f();
        addTransform(affine3f::translate(translate));
        return true;
      }
      if (token == "ConcatTransform") {
        addTransform(parseMatrix());
        return true;
      }
      if (token == "Rotate") {
        const float angle = parseFloat();
        const vec3f axis  = parseVec3f();
        addTransform(affine3f::rotate(axis,angle*(float)M_PI/180.f));
        return true;
      }
      if (token == "Transform") {
        next(); // '['
        affine3f xfm;
        xfm.l.vx = parseVec3f(); float m30 = parseFloat();
        xfm.l.vy = parseVec3f(); float m31 = parseFloat();
        xfm.l.vz = parseVec3f(); float m32 = parseFloat();
        xfm.p = parseVec3f();    float m33 = parseFloat();

        assert(m30 == 0.f && m31 == 0.f && m32 == 0.f);
        // m33 could be not equal to 1.0 (for example, structuresynth\arcsphere.pbrt).
        // We don't support such transformations and corresponding object instances will be discarded.

        next(); // ']'
        addTransform(xfm, std::abs(m33 - 1.f) > 1e-3f);
        return true;
      }
      if (token == "ActiveTransform") {
        std::string time = next().text;
        std::cout << "'ActiveTransform' not implemented" << std::endl;
        return true;
      }
      if (token == "Identity") {
          setTransform(affine3f::identity());
        return true;
      }
      if (token == "ReverseOrientation") {
        /* according to the docs, 'ReverseOrientation' only flips the
           normals, not the actual transform */
        currentGraphicsState->reverseOrientation = !currentGraphicsState->reverseOrientation;
        return true;
      }
      if (token == "CoordSysTransform") {
        Token nameOfObject = next();
        std::cout << "ignoring 'CoordSysTransform'" << std::endl;
        return true;
      }
      return false;
    }

    template <typename DS>
    void BasicParser<DS>::parseWorld()
    {
      while (1) {
        Token token = next();
        assert(token);

        // ------------------------------------------------------------------
        // WorldEnd - go back to regular parseScene
        // ------------------------------------------------------------------
        if (token == "WorldEnd") {
          break;
        }
      
        // -------------------------------------------------------
        // LightSource
        // -------------------------------------------------------
        if (token == "LightSource") {
          std::shared_ptr<LightSource> lightSource
            = std::make_shared<LightSource>(next().text,ctm);
          parseParams(lightSource->param);
          getCurrentObject()->lightSources.push_back(lightSource);
          // attributesStack.top()->lightSources.push_back(lightSource);
          continue;
        }

        // ------------------------------------------------------------------
        // AreaLightSource
        // ------------------------------------------------------------------
        if (token == "AreaLightSource") {
          std::shared_ptr<AreaLightSource> lightSource
            = std::make_shared<AreaLightSource>(next().text);
          parseParams(lightSource->param);
          // getCurrentObject()->lightSources.push_back(lightSource);
          currentGraphicsState->areaLightSources.push_back(lightSource);
          continue;
        }

        // -------------------------------------------------------
        // Material
        // -------------------------------------------------------
        if (token == "Material") {
          std::string type = next().text;
          std::shared_ptr<Material> material
            = std::make_shared<Material>(type);
          parseParams(material->param);
          currentMaterial = material;
          material->attributes = currentGraphicsState->getClone();;
          continue;
        }

        // ------------------------------------------------------------------
        // Texture
        // ------------------------------------------------------------------
        if (token == "Texture") {
          std::string name = next().text;
          std::string texelType = next().text;
          std::string mapType = next().text;
          std::shared_ptr<Texture> texture
            = std::make_shared<Texture>(name,texelType,mapType);
          currentGraphicsState->insertNamedTexture(name, texture);
          texture->attributes = currentGraphicsState->getClone();;
          parseParams(texture->param);
          continue;
        }

        // ------------------------------------------------------------------
        // MakeNamedMaterial
        // ------------------------------------------------------------------
        if (token == "MakeNamedMaterial") {        
          std::string name = next().text;
          std::shared_ptr<Material> material
            = std::make_shared<Material>("<implicit>");
          currentGraphicsState->insertNamedMaterial(name, material);
          parseParams(material->param);
          material->attributes = currentGraphicsState->getClone();;
          
          /* named material have the parameter type implicitly as a
             parameter rather than explicitly on the
             'makenamedmaterial' command; so let's parse this here */
          std::shared_ptr<Param> type = material->param["type"];
          if (!type) throw std::runtime_error("named material that does not specify a 'type' parameter!?");
          std::shared_ptr<ParamArray<std::string>> asString
            = std::dynamic_pointer_cast<ParamArray<std::string> >(type);
          if (!asString)
            throw std::runtime_error("named material has a type, but not a string!?");
          assert(asString->getSize() == 1);
          material->type = asString->get(0); //paramVec[0];
          material->name = name;
          continue;
        }

        // ------------------------------------------------------------------
        // MakeNamedMedium
        // ------------------------------------------------------------------
        if (token == "MakeNamedMedium") {
          std::string name = next().text;
          std::shared_ptr<Medium> medium
            = std::make_shared<Medium>("<implicit>");
          currentGraphicsState->insertNamedMedium(name, medium);
          parseParams(medium->param);

          /* named medium have the parameter type implicitly as a
             parameter rather than explicitly on the
             'makenamedmedium' command; so let's parse this here */
          std::shared_ptr<Param> type = medium->param["type"];
          if (!type) throw std::runtime_error("named medium that does not specify a 'type' parameter!?");
          std::shared_ptr<ParamArray<std::string>> asString
            = std::dynamic_pointer_cast<ParamArray<std::string> >(type);
          if (!asString)
            throw std::runtime_error("named medium has a type, but not a string!?");
          assert(asString->getSize() == 1);
          medium->type = asString->get(0); //paramVec[0];
          continue;
        }

        // ------------------------------------------------------------------
        // NamedMaterial
        // ------------------------------------------------------------------
        if (token == "NamedMaterial") {
          std::string name = next().text;
        
          currentMaterial = currentGraphicsState->findNamedMaterial(name);

          continue;
        }

        // ------------------------------------------------------------------
        // MakeNamedMedium
        // ------------------------------------------------------------------
        if (token == "MakeNamedMedium") {
          std::string name = next().text;
          std::shared_ptr<Medium> medium
            = std::make_shared<Medium>("<implicit>");
          currentGraphicsState->insertNamedMedium(name, medium);
          parseParams(medium->param);

          /* named medium have the parameter type implicitly as a
             parameter rather than explicitly on the
             'makenamedmedium' command; so let's parse this here */
          std::shared_ptr<Param> type = medium->param["type"];
          if (!type) throw std::runtime_error("named medium that does not specify a 'type' parameter!?");
          ParamArray<std::string>::SP asString = type->as<std::string>();
          if (!asString)
            throw std::runtime_error("named medium has a type, but not a string!?");
          assert(asString->getSize() == 1);
          medium->type = asString->get(0); //paramVec[0];
          continue;
        }

        // ------------------------------------------------------------------
        // MediumInterface
        // ------------------------------------------------------------------
        if (token == "MediumInterface") {
          currentGraphicsState->mediumInterface.first = next().text;
          currentGraphicsState->mediumInterface.second = next().text;
          continue;
        }

        // -------------------------------------------------------
        // AttributeBegin
        // -------------------------------------------------------
        if (token == "AttributeBegin") {
          pushAttributes();
          continue;
        }

        // -------------------------------------------------------
        // AttributeEnd
        // -------------------------------------------------------
        if (token == "AttributeEnd") {
          popAttributes();
          continue;
        }

        // -------------------------------------------------------
        // Shape
        // -------------------------------------------------------
        if (token == "Shape") {
          // if (!currentMaterial) {
          //   std::cout << "warning(pbrt_parser): shape, but no current material!" << std::endl;
          // }
          std::shared_ptr<Shape> shape
              = std::make_shared<Shape>(next().text,
                  currentMaterial,
                  currentGraphicsState->getClone(),
                  ctm);
          parseParams(shape->param);
          getCurrentObject()->shapes.push_back(shape);
          continue;
        }
      
        // -------------------------------------------------------
        // Volumes
        // -------------------------------------------------------
        if (token == "Volume") {
          std::shared_ptr<Volume> volume
            = std::make_shared<Volume>(next().text);
          parseParams(volume->param);
          getCurrentObject()->volumes.push_back(volume);
          continue;
        }

        // -------------------------------------------------------
        // Transform
        // -------------------------------------------------------
        if (parseTransform(token))
          continue;
        
        // -------------------------------------------------------
        // ObjectBegin
        // -------------------------------------------------------
        if (token == "ObjectBegin") {
          std::string name = next().text;
          std::shared_ptr<Object> object = findNamedObject(name,1);

          objectStack.push(object);
          continue;
        }
          
        // -------------------------------------------------------
        // ObjectEnd
        // -------------------------------------------------------
        if (token == "ObjectEnd") {
          objectStack.pop();
          continue;
        }

        // -------------------------------------------------------
        // ObjectInstance
        // -------------------------------------------------------
        if (token == "ObjectInstance") {
          std::string name = next().text;

          if (ctm.nonUnitW)
              continue;

          std::shared_ptr<Object> object = findNamedObject(name,1);
          std::shared_ptr<Object::Instance> inst
            = std::make_shared<Object::Instance>(object,ctm);
          getCurrentObject()->objectInstances.push_back(inst);
          continue;
        }
          
        // -------------------------------------------------------
        // ERROR - unrecognized token in worldbegin/end!!!
        // -------------------------------------------------------
        throw std::runtime_error("unexpected token '"+token.toString()
                                 +"' at "+token.loc.toString());
      }
    }

    template <typename DS>
    Token BasicParser<DS>::next()
    {
      Token token = peek();
      if (!token)
        throw std::runtime_error("unexpected end of file ...");
      peekQueue.pop_front();
      return token;
    }
    
    template <typename DS>
    Token BasicParser<DS>::peek(unsigned int i)
    {
      while (peekQueue.size() <= i) {
        Token token = tokens->next();
        // first, handle the 'Include' statement by inlining such files
        if (token && token == "Include") {
          Token fileNameToken = tokens->next();
          std::string includedFileName = fileNameToken.text;
          if (includedFileName[0] != '/') {
            includedFileName = rootNamePath+"/"+includedFileName;
          }
        
          tokenizerStack.push(tokens);
          FileType::SP file = std::make_shared<FileType>(includedFileName);
          if (!replace_tokens(std::make_shared<BasicLexer<FileType>>(file)))
            throw std::runtime_error("incompatible lexers ...");
          continue;
        }
      
        if (token) {
          peekQueue.push_back(token);
          continue;
        }
      
        // last token was invalid, so encountered at least one end of
        // file - see if we can pop back to another one off the stack
        if (tokenizerStack.empty())
          // nothing to back off to, return eof indicator
          return Token();
      
        replace_tokens(tokenizerStack.top());
        tokenizerStack.pop();
        // token = next();
        continue;
      }
      return peekQueue[i];
    }
    
    template <typename DS>
    void BasicParser<DS>::parseScene()
    {
      while (peek()) {
      
        Token token = next();
        if (!token) break;

        // -------------------------------------------------------
        // Transform
        // -------------------------------------------------------
        if (parseTransform(token))
          continue;
        
        if (token == "LookAt") {
          vec3f v0 = parseVec3f();
          vec3f v1 = parseVec3f();
          vec3f v2 = parseVec3f();
        
          // scene->lookAt = std::make_shared<LookAt>(v0,v1,v2);
          affine3f xfm;
          xfm.l.vz = normalize(v1-v0);
          xfm.l.vx = normalize(cross(v2,xfm.l.vz));
          xfm.l.vy = cross(xfm.l.vz,xfm.l.vx);
          xfm.p    = v0;
        
          addTransform(inverse(xfm));
          this->scene->isZAxisUp = std::abs(v2.z) > std::abs(v2.y);
          continue;
        }

        if (token == "ConcatTransform") {
          next(); // '['
          float mat[16];
          for (int i=0;i<16;i++)
            mat[i] = std::stof(next().text);

          affine3f xfm;
          xfm.l.vx = vec3f(mat[0],mat[1],mat[2]);
          xfm.l.vy = vec3f(mat[4],mat[5],mat[6]);
          xfm.l.vz = vec3f(mat[8],mat[9],mat[10]);
          xfm.p    = vec3f(mat[12],mat[13],mat[14]);
          addTransform(xfm);

          next(); // ']'
          continue;
        }
        if (token == "CoordSysTransform") {
          std::string transformType = next().text;
          continue;
        }


        if (token == "ActiveTransform") {
          std::string type = next().text;
          continue;
        }

        if (token == "Camera") {
          std::shared_ptr<Camera> camera = std::make_shared<Camera>(next().text,ctm);
          parseParams(camera->param);
          scene->cameras.push_back(camera);
          continue;
        }
        if (token == "Sampler") {
          std::shared_ptr<Sampler> sampler = std::make_shared<Sampler>(next().text);
          parseParams(sampler->param);
          scene->sampler = sampler;
          continue;
        }
        if (token == "Integrator") {
          std::shared_ptr<Integrator> integrator = std::make_shared<Integrator>(next().text);
          parseParams(integrator->param);
          scene->integrator = integrator;
          continue;
        }
        if (token == "SurfaceIntegrator") {
          std::shared_ptr<SurfaceIntegrator> surfaceIntegrator
            = std::make_shared<SurfaceIntegrator>(next().text);
          parseParams(surfaceIntegrator->param);
          scene->surfaceIntegrator = surfaceIntegrator;
          continue;
        }
        if (token == "VolumeIntegrator") {
          std::shared_ptr<VolumeIntegrator> volumeIntegrator
            = std::make_shared<VolumeIntegrator>(next().text);
          parseParams(volumeIntegrator->param);
          scene->volumeIntegrator = volumeIntegrator;
          continue;
        }
        if (token == "PixelFilter") {
          std::shared_ptr<PixelFilter> pixelFilter = std::make_shared<PixelFilter>(next().text);
          parseParams(pixelFilter->param);
          scene->pixelFilter = pixelFilter;
          continue;
        }
        if (token == "Accelerator") {
          std::shared_ptr<Accelerator> accelerator = std::make_shared<Accelerator>(next().text);
          parseParams(accelerator->param);
          continue;
        }
        if (token == "Film") {
          scene->film = std::make_shared<Film>(next().text);
          parseParams(scene->film->param);
          continue;
        }
        if (token == "Accelerator") {
          std::shared_ptr<Accelerator> accelerator = std::make_shared<Accelerator>(next().text);
          parseParams(accelerator->param);
          continue;
        }
        if (token == "Renderer") {
          std::shared_ptr<Renderer> renderer = std::make_shared<Renderer>(next().text);
          parseParams(renderer->param);
          continue;
        }

        if (token == "WorldBegin") {
          ctm.reset();
          parseWorld();
          continue;
        }

        // ------------------------------------------------------------------
        // MediumInterface
        // ------------------------------------------------------------------
        if (token == "MediumInterface") {
          currentGraphicsState->mediumInterface.first = next().text;
          currentGraphicsState->mediumInterface.second = next().text;
          continue;
        }

        // ------------------------------------------------------------------
        // MakeNamedMedium
        // ------------------------------------------------------------------
        if (token == "MakeNamedMedium") {
          std::string name = next().text;
          std::shared_ptr<Medium> medium
            = std::make_shared<Medium>("<implicit>");
          currentGraphicsState->insertNamedMedium(name, medium);
          parseParams(medium->param);

          /* named medium have the parameter type implicitly as a
             parameter rather than explicitly on the
             'makenamedmedium' command; so let's parse this here */
          std::shared_ptr<Param> type = medium->param["type"];
          if (!type) throw std::runtime_error("named medium that does not specify a 'type' parameter!?");
          std::shared_ptr<ParamArray<std::string>> asString
            = std::dynamic_pointer_cast<ParamArray<std::string> >(type);
          if (!asString)
            throw std::runtime_error("named medium has a type, but not a string!?");
          assert(asString->getSize() == 1);
          medium->type = asString->get(0); //paramVec[0];
          continue;
        }

        if (token == "Material") {
          throw std::runtime_error("'Material' field not within a WorldBegin/End context. "
                                   "Did you run the parser on the 'shape.pbrt' file directly? "
                                   "(you shouldn't - it should only be included from within a "
                                   "pbrt scene file - typically '*.view')");
          continue;
        }

        throw std::runtime_error("unexpected token '"+token.text
                                 +"' at "+token.loc.toString());
      }
    }



#ifdef _WIN32
    const char path_sep = '\\';
#else
    const char path_sep = '/';
#endif

    inline std::string pathOf(std::string fn)
    {
      std::replace(fn.begin(), fn.end(), '\\', '/');
      size_t pos = fn.find_last_of('/');
      if (pos == std::string::npos) {
        return std::string();
      }

      return fn.substr(0,pos+1);
    }


    /*! parse given file, and add it to the scene we hold */
    template <typename DS>
    void BasicParser<DS>::parse(const std::string &fn)
    {
      rootNamePath
        = basePath==""
        ? (std::string)pathOf(fn)
        : (std::string)basePath;
      FileType::SP file = std::make_shared<FileType>(fn);
      this->tokens = std::make_shared<BasicLexer<FileType>>(file);
      parseScene();
      scene->basePath = rootNamePath;
    }

    
    /*! parse from any input stream, add to scene we hold */
    template <typename DS>
    template <typename Stream>
    void BasicParser<DS>::parse(typename IStream<Stream>::SP is)
    {
      this->tokens = std::make_shared<BasicLexer<IStream<Stream>>>(is);
      parseScene();
    }


  } // ::pbrt::syntx
} // ::pbrt
