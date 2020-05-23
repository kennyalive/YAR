#include "std.h"
#include "common.h"
#include "color.h"
#include "vector.h"

//
// sRGB <--> XYZ conversion coefficients are from Bruce Lindbloom's page:
// http://www.brucelindbloom.com/index.html?Eqn_RGB_XYZ_Matrix.html
//
ColorRGB XYZ_to_sRGB(const Vector3& xyz) {
    ColorRGB c;
    c[0] =  3.2404542f * xyz[0] + -1.5371385f * xyz[1] + -0.4985314f * xyz[2];
    c[1] = -0.9692660f * xyz[0] +  1.8760108f * xyz[1] +  0.0415560f * xyz[2];
    c[2] =  0.0556434f * xyz[0] + -0.2040259f * xyz[1] +  1.0572252f * xyz[2];
    return c;
}

ColorRGB sRGB_to_XYZ(const ColorRGB& rgb) {
    ColorRGB xyz;
    xyz[0] = 0.4124564f * rgb[0] + 0.3575761f * rgb[1] + 0.1804375f * rgb[2];
    xyz[1] = 0.2126729f * rgb[0] + 0.7151522f * rgb[1] + 0.0721750f * rgb[2];
    xyz[2] = 0.0193339f * rgb[0] + 0.1191920f * rgb[1] + 0.9503041f * rgb[2];
    return xyz;
}
