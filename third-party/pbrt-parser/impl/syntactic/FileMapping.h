// Copyright 2015-2019 Ingo Wald
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

/*! namespace for all things pbrt parser, both syntactical *and* semantical parser */
namespace pbrt {
  /*! namespace for syntactic-only parser - this allows to distringuish
    high-level objects such as shapes from objects or transforms,
    but does *not* make any difference between what types of
    shapes, what their parameters mean, etc. Basically, at this
    level a triangle mesh is nothing but a geometry that has a string
    with a given name, and parameters of given names and types */
  namespace syntactic {

    class FileMapping {
      void *mapping;
      size_t num_bytes;
#ifdef _WIN32
      HANDLE file;
      HANDLE mapping_handle;
#else
      int file;
#endif

    public:
      // Map the file into memory
      FileMapping(const std::string &fname);
      FileMapping(FileMapping &&fm);
      ~FileMapping();
      FileMapping& operator=(FileMapping &&fm);

      FileMapping(const FileMapping &) = delete;
      FileMapping& operator=(const FileMapping&) = delete;

      const uint8_t* data() const;
      size_t nbytes() const;
    };

    template<typename T>
    class BasicStringView {
      const T *ptr;
      size_t count;

      public:
      BasicStringView() : ptr(nullptr), count(0) {}

      /* Create a typed view into a string. The count is in
       * number of elements of T in the view.
       */
      BasicStringView(const T* ptr, size_t count)
          : ptr(ptr), count(count)
      {}
      const T& operator[](const size_t i) const {
          return ptr[i];
      }
      const T* data() const {
          return ptr;
      }
      size_t size() const {
          return count;
      }
      const T* cbegin() const {
          return ptr;
      }
      const T* cend() const {
          return ptr + count;
      }
    };

    typedef BasicStringView<char> StringView;
  } // ::syntactic
} // ::pbrt
