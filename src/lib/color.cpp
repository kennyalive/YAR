#include "common.h"
#include "color.h"
#include "vector.h"

// XYZ->sRGB conversion coefficients are from http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
ColorRGB ColorRGBFromXYZ(const Vector3& xyz) {
    ColorRGB c;
    c[0] =  3.2404542f * xyz[0] + -1.5371385f * xyz[1] + -0.4985314f * xyz[2];
    c[1] = -0.9692660f * xyz[0] +  1.8760108f * xyz[1] +  0.0415560f * xyz[2];
    c[2] =  0.0556434f * xyz[0] + -0.2040259f * xyz[1] +  1.0572252f * xyz[2];
    return c;
}
