// Copyright 2015-2019 Ingo Wald
// SPDX-License-Identifier: Apache-2.0

/*! namespace for all things pbrt parser, both syntactical *and* semantical parser */
namespace pbrt {
  /*! namespace for syntactic-only parser - this allows to distringuish
    high-level objects such as shapes from objects or transforms,
    but does *not* make any difference between what types of
    shapes, what their parameters mean, etc. Basically, at this
    level a triangle mesh is nothing but a geometry that has a string
    with a given name, and parameters of given names and types */
  namespace syntactic {

    // =======================================================
    // ReadBuffer<DataSource>
    // =======================================================

    template <typename DS>
    ReadBuffer<DS>::ReadBuffer(DS s) : source(s) {}

    template <typename DS>
    void ReadBuffer<DS>::unget_char(int c) {
      if (peekBuffer[0] >= 0)
        throw std::runtime_error("can't push back more than one char ...");
      peekBuffer[0] = c;
      line = lineBuffer[0];
      col = colBuffer[0];
    }

    template <typename DS>
    int ReadBuffer<DS>::get_char() {

      int c;

      if (peekBuffer[0] >= 0) {
        c = peekBuffer[0];
        peekBuffer[0] = -1;
      } else {
        c = source->get();
      }

      // Loc
      lineBuffer[0] = line;
      colBuffer[0] = col;
      if (c == '\n') {
        line++;
        col = 0;
      } else {
        col++;
      }

      return c;
    }

    namespace detail {
      // disambiguate File::SP and IStream::SP, only File has valid ptr
      template <typename DS> inline std::shared_ptr<File> filePointer(const DS& ptr) { return nullptr; }
      inline std::shared_ptr<File> filePointer(const std::shared_ptr<File>& ptr) { return ptr; }
    }

    template <typename DS>
    Loc ReadBuffer<DS>::get_loc() const {
      return { detail::filePointer(source), line, col };
    }
  } // ::syntactic
} // ::pbrt
